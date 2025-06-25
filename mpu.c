/*

@name mpu.c
@brief Driver para el MPU6050. Se definene las funciones de inicialización y de desisnstalación del platform driver. Se configura el periférico I2C del Sitara335x. Se configura el modulo MPU6050
@author Ian Sztenberg
@date 09-2022
@last modified 02-12-2022
@version 1.0


Muchos de los comentarios realizados en este código fueron tomados de explicacinones brindadas en "Linux Device Drivers 3rd" - Jonathan Corbet, Alessandro Rubini y Greg Kroah-Hartman
	Dicho libro puede accederse en este link y está disponible de manera libre:	https://lwn.net/Kernel/LDD3/

*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Ian Szteberg");
MODULE_DESCRIPTION("Escribir un caracter al device driver. Luego en las siguientes lecturas el device driver genera caracteres con codigo ASCII creciente. Si se escribe la letra a, la lectura hace que el proceso se vaya a dormir hasta que otro proceso escriba una letra mayúscula.");

//declaro cola de espera
DECLARE_WAIT_QUEUE_HEAD (wait_queue);


//registros driver
#define CM_PER 					0x44E00000  // Clock Module Peripheral Registers --> página 179 manual Sitara 335x
#define CM_PER_SIZE 			1024			//tamaño 1KB	
#define CM_PER_I2C2_CLK_OFFSET 	0x44		// página 1253 manual Sitara 335x. El offset 44h dentro del Periférico CM_PER, corresponde al clock del I2C2- Bits 1-0 = MODULEMODE = 2h (module explicity enabled) - Bits 17-16 = IDLEST = 0h (Module is fully functional) 
#define I2C2 					0x4819C000				//manual del Sitara 335x página 183. Tamaño: 4KB (0x4819CFFF)
#define I2C2_MODULE_SIZE 		4096		//manual del Sitara 335x página 183. Tamaño: 4KB (0x4819CFFF)
#define I2C2_REV 				0x0

//control module
#define CONTROL_MODULE			0x44E10000  //manual Sitara 335x página 180. Tamaño 128KB (0x44E11FFF)
#define CONTROL_MODULE_SIZE		131072		//manual Sitara 335x página 180. Tamaño 128KB (0x44E11FFF)
#define	I2C2_SDA_OFFSET			0x978				
#define I2C2_SCL_OFFSET			0x97C


//registros configuración I2C
#define I2C2_PSC_OFFSET			0xB0 //se usan los primeros 8 bits, los demás están reservados. 0x0 = divide por 1. 0x1 = divide por 2. ... 0xff = divider por 256	(SCLK / (1+PSC)) 
#define I2C2_SCLL_OFFSET		0xB4 //se usan los primeros 8 bits, los demás están reservados. tLOW  = (SCLL + 7) * ICLK
#define I2C2_SCLH_OFFSET		0xB8 //se usan los primeros 8 bits, los demás están reservados. tHIGH = (SCLH + 5) * ICLK
#define I2C2_CON_OFFSET			0xA4 //	 P.4634 Manual sitara
#define I2C_IRQENABLE_OFFSET	0x2C //P4614 Manual Sitara
#define I2C_SA_OFFSET			0xAC 
#define I2C_CNT_OFFSET			0x98
#define I2C_IRQSTATUS_RAW	 	0x24 //
#define I2C_DATA_OFFSET	 		0x9C // registro de 1 byte donde tenemos la info a Tx o Rx. La info a transmitir puede escribirse en esta 
#define I2C_OA_OFFSET	 		0xA8  //
#define I2C_SYSTEST_OFFSET	 	0xBC //permite realizar pruebas sobre el modulo I2C, forzando salida del CLK por el pin SCL	
#define I2C_IRQENABLE_CLR		0x30 //registro para limpiar el IRQENABLE
#define I2C_IRQSTATUS 			0x28
#define I2C_BUFFER_OFFSET 		0x94 //registro utilizado para confiurar el valor umbral de la FIFO de Tx: cuando se alcanza el valor umbral, se comienza la transferencia. 


//registros configuracion MPU6050
#define MPU_CONFIG_OFFSET		0x1A
#define MPU_GYR_CONFIG_OFFSET	0x1B
#define MPU_ACC_CONFIG_OFFSET	0x1C
#define MPU_FIFO_EN_OFFSET 		0x23
#define MPU_ACC_ZH_OFFSET 		0x3F
#define MPU_ACC_ZL_OFFSET 		0x40
#define MPU_ACC_YH_OFFSET 		0x3D
#define MPU_ACC_YL_OFFSET 		0x3E
#define MPU_ACC_XH_OFFSET 		0x3B
#define MPU_ACC_XL_OFFSET 		0x3C
#define MPU_PWRMGMT_OFFSET		0x6B
#define MPU_TEMPH_OFFSET 		0x41	
#define MPU_TEMPL_OFFSET 		0x42
#define MPU_FIFOCH_OFFSET 		0x72
#define MPU_FIFOCL_OFFSET 		0x73
#define MPU_FIFORW_OFFSET		0x74
#define MPU_TSTX_OFFSET 		0x0D //registros de SELF TEST : muestran el resultado de los self-test
#define MPU_TSTY_OFFSET 		0x0E
#define MPU_TSTZ_OFFSET 		0x0F
#define MPU_TST0_OFFSET 		0x10 //bits 1-0 de los bits de self test
#define MPU_USRCTRL_OFFSET 		0x6A //user control: permite habilitad la FIFO, i2c master mode y la interfaz i2c primaria
#define MPU_WHOAMI_OFFSET		0x75

//funcion mpu_read
#define paq_size 6


static dev_t dev;	//dev_t es parte de la estrcutura cdev
static struct class *cl; 



static int funcion_probe(struct platform_device * pdev);
static int funcion_remove(struct platform_device * pdev);
static irqreturn_t Mi_handler_interrupt(int irq);


static int i2c_send(int raddr,int data);
static uint8_t i2c_read(int raddr);

//static int i2c_data[2]; 

static struct{
    int irq;
    void *cm_per;
    void *i2c2;
    void *mode;    
}Host_Controller;


//variables globales
static int i2c_con = 0;
static int i2c_bb = 0;
static int i2c_buf = 0;
static int i2c_xrdy = 0;
static int i2c_ardy = 0;
static int i2c_nack = 0;
static int i2c_rrdy = 0;
static char ardy_flag = 0;
static char xrdy_flag = 0;
static char nack_flag = 0;
static char rrdy_flag = 0;
//int i2c_systst = 0;
static int i2c_irq_status = 0;
static uint8_t i2c_data = 0;

//variables usadas para almacenar los datos leídos del sensor acelerómetro
//static uint8_t AXL = 0;
//static uint8_t AXH = 0;
//static uint8_t AYL = 0;
//static uint8_t AYH = 0;
//static uint8_t AZL = 0;
//static uint8_t AZH = 0;

static uint8_t FIFOCH = 0;
static uint8_t FIFOCL = 0;
static uint16_t FIFOC = 0;

//static int16_t AZ = 0;
//static int16_t AY = 0;
//static int16_t AX = 0;
//static int16_t AXF = 0;
//static int16_t AYF = 0;
//static int16_t AZF = 0;
//static int i = 0;
/*
	Para que el kernel sepa qué HW debe manejar nuestro driver, tenemos que asociar nuestro platform_driver con el platform_device. Para esto: 	
		1) guardamos en una struct los dispos compatibles con nuestro platform_driver, mediante una tala tipo "struct of_device_id"
*/
static const struct of_device_id mis_dispos_compatibles [] = {
	{.compatible = "td3-i2c"},
		{ },
};

/*
	Las funciones asociadas a un "platform_driver" para controlar un "platform_device" se encuentran en la "struct platform_driver"  => definimos una instancia de esta estructura para registrar nuestro platform_driver
*/
static struct platform_driver Mi_I2C_Host_Controller = {
        .probe = funcion_probe,
        .remove = funcion_remove,
        .driver = {
            .name = "Host_Controller",
            .of_match_table = of_match_ptr(mis_dispos_compatibles),
        },    
    
};

/*
	Informamos al kernel de los posibles dispositivos (HW) que podrían conectarse al sistema. Una vez le informamos al kernel sobre el HW compatible, este buscará una coincidencia entre el campo compatible de un nodo del 
		device tree y y el campo compatible de la estructura of_device_id	
*/
MODULE_DEVICE_TABLE(of,mis_dispos_compatibles);


/*
	Probe(): es la función que se llamará cuando se produzca una coincidencia entre el campo compatible del dispositivo descripto en el device_tree y el campo compatible que especificamos en la estructura of_device_id
	
	Dentro de las tareas de probe se encuentran:
		1)Check whether the device is the one you expected
		2)Check whether your I2C bus controller of the SoC supports the functionality needed by your device, using the i2c_check_functionality function
		3)Initialize the device <-- tenemos que inicializar
		4)Set up device-specific data
		5)Register the appropriate kernel framework

	
*/
static int funcion_probe(struct platform_device * pdev)
{
        static int Request_result;
		
        printk(KERN_ALERT "Ingreso a PROBE\n");
    
		/*
			El kernel de Linux maneja dispositivos que pueden ser definidos como controladores de interrupciones. A cada interrupción se le asigna una VIRQ, el cual usualmente no coincide con la línea de interrupción. 
				La línea de interrupción se define en el Device Tree y en la función probe se pide una VIRQ para esa interrupción a través de la siguiente función:
				
					int platform_get_irq (struct platform_device *, uint)
				
				
					Se le debe pasar el puntero recibido en la función probe y un índice por si hay más de una INT (usualmente es 0). Devuelve una VIRQ
					
					Una vez que se tiene el VIRQ ya se puede implantar el handler de IRQ. El handler deberá tener el siguiente prototipo: 
					
						irqreturn t mi_handler (int irq, void *dev id, struct pt regs*regs)
						
						Siemrpe se debe volver de mi_handler, devolvindo la cosntante IRQ_HANDLED, para informar que el kernel manejó la interrupción correctamente 
						
						
						
					Para implantar handler se deben usar las siguientes funciones: 
					
						int request_irq (unsigned_int virq, irq_handler_t mi_handler, unsigned_long irqflags, const char * devname, void * dev_id);
						
						void free irq (unsigned_int irq, void * dev_id);
						
						dev_id = NULL 
						irqflags = IRQF TRIGGER RISING
						
						
						
						
		*/
	
		//Para obtener una línea de interrupción disponible, instanciar el método
        Host_Controller.irq = platform_get_irq(pdev,0);
		//para asociar la ISR correspondiente con cada IRQ
		
		/*
			int request_irq(unsigned int irq, irq_handler_t handler, unsigned long flags, const char *name, void *dev) -> Add a handler for an interrupt line.
			
			parametros
				unsigned int irq

				The interrupt line to allocate
				irq_handler_t handler

				Function to be called when the IRQ occurs. Primary handler for threaded interrupts If NULL, the default primary handler is installed
				unsigned long flags

				Handling flags
				const char *name

				Name of the device generating this interrupt
				void *dev

				A cookie passed to the handler function

				This call allocates an interrupt and establishes a handle
			
		*/
        Request_result = request_irq ( Host_Controller.irq,(irq_handler_t)Mi_handler_interrupt,IRQF_TRIGGER_RISING,pdev->name,NULL );
        
		
		
        if(Request_result < 0){
            printk(KERN_ALERT "Mi_IRQ ERROR \n");
            return -1;
        }
        
        //I2C clock domain configuration
        Host_Controller.cm_per = ioremap (CM_PER,CM_PER_SIZE);
        if(Host_Controller.cm_per == NULL)
		{
            printk(KERN_ALERT "CM PER ERROR \n");
            return -1;
        }
		
		// página 1253 manual Sitara 335x. El offset 44h dentro del Periférico CM_PER, corresponde al clock del I2C2- 
		//	 Bits 1-0 = MODULEMODE = 2h (module explicity enabled) 
		//	 Bits 17-16 = IDLEST = 0h (Module is fully functional) 
		
		// => en el offset 44h, escrubumos un 0x02 => IDLEST = 0h y MODULEMODE = 2h
        iowrite32 (0x02, Host_Controller.cm_per + CM_PER_I2C2_CLK_OFFSET); //--> estamos habiliando el clock del I2C2
        printk(KERN_ALERT "Llego a escribir CMPER - I2C2_CLKCTR %d\n",ioread32(Host_Controller.cm_per + CM_PER_I2C2_CLK_OFFSET));
        printk(KERN_ALERT "Ya escribí CM_PER y habilité el clock del I2C2 \n");
        
   
		/*
			Control module: seleccionamos la funcionalidad I2C para los pines 19 y 20 del P9
				
			SDA offset 978h: 
							bit 31-7: rsv
							bit 6: slewrate (0 -> fast)
							bit 5: rxactive (1 -> rx_enabled)
							bit 4: Pullup or down (1 -> up)
							bit 3: Pullup/down enable (0 -> enabled)
							bit 2-0: 011 (function 3 = I2C SDA)
							
								
			SCL offset 97Ch: 
		
		*/
		Host_Controller.mode = ioremap (CONTROL_MODULE,CONTROL_MODULE_SIZE);
        if(Host_Controller.mode == NULL)
		{
            printk(KERN_ALERT "CONTROL MODULE ERROR \n");
            return -1;
        }
		iowrite32 (0x33, Host_Controller.mode + I2C2_SDA_OFFSET); //--> elejimos el modo del pin
		printk(KERN_ALERT "Llego a escribir CONTROL MODULE - SDA %d\n",ioread32(Host_Controller.mode + I2C2_SDA_OFFSET));
		iowrite32 (0x33, Host_Controller.mode + I2C2_SCL_OFFSET); //--> elejimos el modo del pin
		printk(KERN_ALERT "Llego a escribir CONTROL MODULE - SCL %d\n",ioread32(Host_Controller.mode + I2C2_SCL_OFFSET));
		
		
		printk(KERN_ALERT "Ya escribí elegí el modo I2C2 para los pines 19 y 20 de P9 \n");
		
        //i2c2
		/*
			Así como habilitamos el clock, ahora tenemos que configuar el modulo en sí. 
			
			
			Del Technical reference del Sitara (p. 4599)
			
			Module Configuration Before Enabling the Module
				1. Program the prescaler to obtain an approximately 12-MHz I2C module clock (I2C_PSC = x; this value
				is to be calculated and is dependent on the System clock frequency).
				2. Program the I2C clock to obtain 100 Kbps or 400 Kbps (SCLL = x and SCLH = x; these values are to
				be calculated and are dependent on the System clock frequency).
				3. Configure its own address (I2C_OA = x) - only in case of I2C operating mode (F/S mode).
				4. Take the I2C module out of reset (I2C_CON:I2C_EN = 1)
				
			Initialization Procedure
				1. Configure the I2C mode register (I2C_CON) bits.
				2. Enable interrupt masks (I2C_IRQENABLE_SET), if using interrupt for transmit/receive data.
				3. Enable the DMA (I2C_BUF and I2C_DMA/RX/TX/ENABLE_SET) and program the DMA controller) -
				only in case of I2C operating mode (F/S mode), if using DMA for transmit/receive data
				
				
			Configure Slave Address and DATA Counter Registers
				In master mode, configure the slave address (I2C_SA = x) and the number of byte associated with the
				transfer (I2C_CNT = x)	
			
			
		*/
	
        Host_Controller.i2c2 = ioremap (I2C2,I2C2_MODULE_SIZE);
        if(Host_Controller.i2c2 == NULL){
            printk(KERN_ALERT "I2C2 ERROR \n");
            return -1;
        }
		
		
		/*
			configuro pre-scaler del I2C2
			
			Se usan los primeros 8 bits, los demás están reservados. 0x0 = divide por 1. 0x1 = divide por 2. ... 0xff = divider por 256	(SCLK / (1+PSC)) 
			
			El clock que reibimos es 192MHz dividio 4 => 48MHz
			
			Freq = SCLK / (1+PSC)

			(1+PSC) = SCLK / Freq
			
			PSC = SCLK / Freq -1 			Freq = 12MHz SCLK = 19.2MHz
			
			PSC = 48 / 12 - 1	
				
			PSC = 3
			
			
			Si SCLK = 48MHz => PSC = 3h
			
			
		*/
        iowrite32 (0x03, Host_Controller.i2c2 + I2C2_PSC_OFFSET); //--> estamos habiliando el clock del I2C2
		printk(KERN_ALERT "Llego a escribir I2C2 - PSC %d\n",ioread32(Host_Controller.i2c2 + I2C2_PSC_OFFSET));
		
		/*
			configuro el duty cycle del I2C configurando SCLL y SCLH
			
			
			
			Se usan los primeros 8 bits, los demás están reservados.  tLOW = (SCLL + 7) * ICLK
			
			
			
			
			Leído					Escrito

SCLL		61h = 0110 0001b		3dh = 0011 1101
SCLH		50h = 0101 0000b		32h = 0011 0010

		*/
        iowrite32 (0x09, Host_Controller.i2c2 + I2C2_SCLL_OFFSET ); //--> estamos habiliando el clock del I2C2		//3Dh = 61d
		printk(KERN_ALERT "Llego a escribir I2C2 - SCLL %d\n",ioread32(Host_Controller.i2c2 + I2C2_SCLL_OFFSET));
		
		/*
			configuro SCLH del I2C2
			
			 Se usan los primeros 8 bits, los demás están reservados.  tHIGH = (SCLH + 5) * ICLK   Donde ICLK = 12MHz
			El registro está partido en dos
			
			
			Para el modo Standar, el duty cycle es del 45,977% (aprox) (tomado de especificación del bus I2C )
			
			tLOW + tHIGH = 1/(100KHz) (standard mode)
			
			
		====>	
			
			I2SCLH & I2SCLL: The values in these two registers will be used to set the data rate of the I2C communication. The bit frequency is given the formula

				Bit Frequency = Fclk / (I2SCLH+I2SCLL)
			
		*/
        iowrite32 (0x08, Host_Controller.i2c2 + I2C2_SCLH_OFFSET ); //--> estamos habiliando el clock del I2C2		//32h = 50d
		printk(KERN_ALERT "Llego a escribir I2C2 - SCLH %d\n",ioread32(Host_Controller.i2c2 + I2C2_SCLH_OFFSET));
		
		
		
		/*
			Tenemos que configurar el slave address
			I2C_SA:
				Bit 31-10: rsv (0)
				Bit 9-0: slave address 0001101000
			
			
			I2C_CNT:
				Bit 31-16: rsv
				Bit 15-0: DCOUNT
		*/
		iowrite32 (0x68, Host_Controller.i2c2 + I2C_SA_OFFSET ); //--> Habilitamos y configuramos el modo de funcionamiento del I2C. Slave address
		
		iowrite32 (0x01, Host_Controller.i2c2 + I2C_CNT_OFFSET); //--> Habilitamos y configuramos el modo de funcionamiento del I2C. Cantidad de datos a transmitir
		
		/*
			Configurar la dirección del master (I2C_OA)
			
			I2C_OA = 1010 0001 = 0xA1
		*/
		
		//iowrite32 (0xA1, Host_Controller.i2c2 + I2C_OA_OFFSET ); //--> Habilitamos y configuramos el modo de funcionamiento del I2C
		 printk(KERN_ALERT "Llego a escribir I2C2 - OA %d\n",ioread32(Host_Controller.i2c2 + I2C_OA_OFFSET));
		/*
			Take the I2C module out of reset
			I2C_CON:I2C_EN = 1
			
			I2C_EN es el bit 15 del registro I2C_CON
			
			Bits 31-16: rsv (0)
			Bit 15: Enable (1)
			Bit 14: rsv (0)
			Bit 13-12: OPMODE (00)
			Bit 11: StartByte (0)
			Bit 10: Master (1)
			Bit 9: TRX (1)
			Bit 8: XSA (0) (MPU 7 bit address)
			Bit 7: XOA0 (0) (MPU 7 bit address)
			Bit 6: XOA1 (0) (MPU 7 bit address)
			Bit 5: XOA2 (0) (MPU 7 bit address)
			Bit 4: XOA3 (0) (MPU 7 bit address)
			Bit 3-2: rsv (00)
			Bit 1: STP (0) (no action or stop condition detected)
			Bit 0: STT (0) (no action or start condition detected)
		*/
		iowrite32 (0x8600, Host_Controller.i2c2 + I2C2_CON_OFFSET ); //--> Habilitamos y configuramos el modo de funcionamiento dle I2C //8600h = 34304d

		
		printk(KERN_ALERT "\n El I2C2_CON es %d", ioread32(Host_Controller.i2c2 + I2C2_CON_OFFSET));
		
		
		/*	
			El I2C_SYSTEST permite forzar el CLK (SCL). Se lo setea en modo "libre" o contínuo para que funcione sin detenerse. Sólo para pruebas
		*/
		
//		iowrite32 (0x8000, Host_Controller.i2c2 + I2C_SYSTEST_OFFSET ); //--> Habilitamos y configuramos el modo de funcionamiento dle I2C //8600h = 34304d
//		printk(KERN_ALERT "\n El I2C2_SYSTESTS es %x", ioread32(Host_Controller.i2c2 + I2C_SYSTEST_OFFSET));
//		
//		
//		i2c_systst = ioread32(Host_Controller.i2c2 + I2C_SYSTEST_OFFSET);
//		i2c_systst = i2c_systst | 0x2000;  
//		iowrite32 (i2c_systst, Host_Controller.i2c2 + I2C_SYSTEST_OFFSET ); //--> Habilitamos y configuramos el modo de funcionamiento dle I2C //8600h = 34304d
//		printk(KERN_ALERT "\n El I2C2_SYSTESTS es %x", ioread32(Host_Controller.i2c2 + I2C_SYSTEST_OFFSET));
		

		
		/*
			Ahora hay que habilitar las interrupciones.
			I2C_IRQENABLE_SET
			
			Bit 31-15: rsv (0)
			Bit 14: (0)
			Bit 13: (0)
			Bit 12: (0)
			Bit 11: (0)
			Bit 10: (0)
			Bit 9: (0)
			Bit 8: BF_IE (1)
			Bit 7: (0) 
			Bit 6: (0)
			Bit 5: (0)
			Bit 4: XRDY_IE (1)
			Bit 3: RRDY_IE (1)
			Bit 2: ARDY_IE (1)
			Bit 1: (0)
			Bit 0: (0)
			
		*/
		//iowrite32 (0x11C, Host_Controller.i2c2 + I2C_IRQENABLE_OFFSET ); //--> Habilitamos y configuramos el modo de funcionamiento dle I2C
        printk(KERN_ALERT "Llego a escribir I2C2 - IRQ I2C2 %d\n",ioread32(Host_Controller.i2c2 + I2C_IRQENABLE_OFFSET));
        
		/*
			Tenemos que configurar el slave address
			I2C_SA:
				Bit 31-10: rsv (0)
				Bit 9-0: slave address 0001101000
			
			
			I2C_CNT:
				Bit 31-16: rsv
				Bit 15-0: DCOUNT
		*/
		//iowrite32 (0x68, Host_Controller.i2c2 + I2C_SA_OFFSET ); //--> Habilitamos y configuramos el modo de funcionamiento del I2C
		
		//iowrite32 (0x00, Host_Controller.i2c2 + I2C_CNT_OFFSET); //--> Habilitamos y configuramos el modo de funcionamiento del I2C
		
		
		
		
		/*
			Pasos para send: 
				1) DCOUNT 
				2) DATA
				3) habilitar interrupción
				4) STT = 1 
		*/
		


/******************************** PRUEBAS DE ENVÍO DE DATOS*************************************/		
		//Marco el bus como ocupado para probar
		
//       	if( (i2c_send(0x00,0x01)) == 0 )
//		{
//			printk(KERN_ALERT "Envió exitoso");
//		}
//	   i2c_bb = ioread32(Host_Controller.i2c2 + I2C_IRQSTATUS_RAW);
//	   i2c_bb = i2c_bb & 0x8000;
//	   i2c_bb = i2c_bb >> 11;
//	   
//	   printk(KERN_ALERT "BB = %x",i2c_bb);
//	   if( i2c_bb == 0) //si el BB está en 0
//	   {
//		   
//			//configuro DCOUNT
//			printk(KERN_ALERT "Configuro DCOUNT");
//			iowrite32 (0x01, Host_Controller.i2c2 + I2C_CNT_OFFSET);
//		   
//			printk(KERN_ALERT "Cargo DATA");
//			iowrite32 (0x01, Host_Controller.i2c2 + I2C_DATA_OFFSET); 
//			printk(KERN_ALERT "Llego a escribir I2C2 - I2C2 DATA %d\n",ioread32(Host_Controller.i2c2 + I2C_DATA_OFFSET));
//		   
//			i2c_bb = ioread32(Host_Controller.i2c2 + I2C_IRQSTATUS_RAW);
//			i2c_bb = i2c_bb & 0x1000;
//			i2c_bb = i2c_bb >> 12;
//	   
//			printk(KERN_ALERT "%x",i2c_bb);
//		   
//			printk(KERN_ALERT "Habilito ISR");
//			iowrite32 (0x10, Host_Controller.i2c2 + I2C_IRQENABLE_OFFSET ); //--> Habilitamos y configuramos el modo de funcionamiento dle I2C
//			printk(KERN_ALERT "Llego a escribir I2C2 - IRQ I2C2 %d\n",ioread32(Host_Controller.i2c2 + I2C_IRQENABLE_OFFSET));
//			
//			
//			printk(KERN_ALERT "STT = 1");
//			iowrite32 (0x8601, Host_Controller.i2c2 + I2C2_CON_OFFSET );		   
//		   
//		   //iowrite32 (0x8601, Host_Controller.i2c2 + I2C2_CON_OFFSET ); //--> Habilitamos y configuramos el modo de funcionamiento dle I2C //8600h = 34304d
//		   //printk(KERN_ALERT "Pongo en 1 el I2C2_CON.STT");
//	   }
	   
	   printk(KERN_ALERT "Volví de la IRQ!");
	   if(ardy_flag)
	   {
		   ardy_flag = 0;
			printk(KERN_ALERT "limpié ardy flag!");
	   }
//		iowrite32 (0x8601, Host_Controller.i2c2 + I2C2_CON_OFFSET ); //--> Habilitamos y configuramos el modo de funcionamiento dle I2C //8600h = 34304d
//		printk(KERN_ALERT "Pongo en 1 el I2C2_CON.STT");
//		
//		//cargo el registro I2C_DATA, que es la FIFO de RX/TX
//		iowrite32 (0x01, Host_Controller.i2c2 + I2C_DATA_OFFSET); 
//        printk(KERN_ALERT "Llego a escribir I2C2 - I2C2 DATA %d\n",ioread32(Host_Controller.i2c2 + I2C_DATA_OFFSET));
		
		printk(KERN_ALERT "Leo BB luego de cargar FIFO - I2C2 BB %d\n",ioread32(Host_Controller.i2c2 + I2C_IRQSTATUS_RAW));
		
		/*
			En el modo de interrupción FIFO, el procesadr es informado del estado del FIFO Rx y Tx mediante interrupciones. Estas interrupciones se disparan cuando se alcanza el valor 
				umbral de la FIFO de Rx/Tx (I2C_BUF.TXTRSH). Estas señales de interrupción indican al host local que transifera información al destino (de la fuenta de info a la FIFO de Tx, en el caso de tranmisión)
		*/

	

/******************************** CONFIGURACION MPU6050*************************************/		

	//el sensor amanece en sleep (0x040). Lo escribo en 0x00 para sacarlo de ese modo.
	i2c_send(MPU_PWRMGMT_OFFSET,0x00);
	/*
		configura el pin externo FSYNC (no utilizado) y el DLPF que se aplicará a los datos del acelerómetro y giróscopo
		
		Bit 7: -
		Bit 6: -
		Bit 5 - 3: EXT_SYNC_SET (0)
		Bit 2 - 0: DLPF 

	*/
	i2c_send(MPU_CONFIG_OFFSET,0x06); 
	
	/*
		configura el giroscopio: rango y self_test 
		
		Bit 7: XG_ST
		Bit 6: YG_ST
		Bit 5: ZG_ST
		Bit 4 - 3: FS_SEL
		Bit 2 - 0: RSV 

	*/
	i2c_send(MPU_GYR_CONFIG_OFFSET,0x0);
	
	/*
		configura el giroscopio: rango y self_test 
		
		Bit 7: XG_ST
		Bit 6: YG_ST
		Bit 5: ZG_ST
		Bit 4 - 3: FS_SEL
		Bit 2 - 0: RSV 

	*/
	i2c_send(MPU_ACC_CONFIG_OFFSET,0x00); //0x20 habilita el ZA_TEST 
	
	
	
	/*
		permite habilitad la FIFO, i2c master mode y la interfaz i2c primaria.
		Bit 7: rsv 				(0)
		Bit 6: FIFO_EN 			(1)
		Bit 5: I2C_MST_EN 		(0)
		Bit 4: I2C_IF_DIS 		(0)
		Bit 3: rsv 				(0)	
		Bit 2: FIFO_RESET 		(0)
		Bit 1: I2C_MST_RESTET 	(0)		
		Bit 0: SIG_COND_RST 	(0)
	*/	
	i2c_send(MPU_USRCTRL_OFFSET,0x40);
	printk(KERN_ALERT "EL USRCTRL es: %x",i2c_read(MPU_USRCTRL_OFFSET));
	
	
	
	/*
		Habilita el uso de la FIFO interna (1024 bytes) para cada sensor para el que se lo habilite
		Bit 7: TEMP 	(1)
		Bit 6: XG 		(1)
		Bit 5: YG 		(1)
		Bit 4: ZG 		(1)
		Bit 3: ACCEL 	(1)	
		Bit 2: SLV2 	(0)
		Bit 1: SLV1 	(0)		
		Bit 0: SLV0 	(0)
	*/
	i2c_send(MPU_FIFO_EN_OFFSET,0x08); //--> sólo acelerómetro´
	if(ardy_flag)
	{
	   ardy_flag = 0;
		printk(KERN_ALERT "limpié ardy flag después de habiltiar FIFO!");
	}
//	printk(KERN_ALERT "Leí del MPU %d",i2c_read(MPU_FIFO_EN_OFFSET));
		//pongo en 1 el bit de Start
	
	   return 0;
	
}



static int i2c_send(int raddr,int data)
{
	i2c_bb = ioread32(Host_Controller.i2c2 + I2C_IRQSTATUS_RAW);
	i2c_bb = i2c_bb & 0x8000;
	i2c_bb = i2c_bb >> 11;
	if( i2c_bb == 0) //si el BB está en 0
	   {
		   
			//configuro DCOUNT
			printk(KERN_ALERT "Configuro DCOUNT");
			iowrite32 (0x02, Host_Controller.i2c2 + I2C_CNT_OFFSET);
		   
			//configuro el valor umbral del BUFFER. 0x00 = 1 , 0x01 = 2, etc
			printk(KERN_ALERT "Configuro BUFFER I2C");
			iowrite32 (0x01, Host_Controller.i2c2 + I2C_BUFFER_OFFSET);
		   
			printk(KERN_ALERT "Cargo DATA");
			

			
			printk(KERN_ALERT "Llego a escribir I2C2 - I2C2 DATA %x\n",ioread32(Host_Controller.i2c2 + I2C_DATA_OFFSET));
		   
			i2c_bb = ioread32(Host_Controller.i2c2 + I2C_IRQSTATUS_RAW);
			i2c_bb = i2c_bb & 0x1000;
			i2c_bb = i2c_bb >> 12;
	   
			printk(KERN_ALERT "%x",i2c_bb);
		   
			printk(KERN_ALERT "Habilito ISR");
			iowrite32 (0x10, Host_Controller.i2c2 + I2C_IRQENABLE_OFFSET ); //--> Habilitamos y configuramos el modo de funcionamiento del I2C
			printk(KERN_ALERT "Llego a escribir I2C2 - IRQ I2C2 %d\n",ioread32(Host_Controller.i2c2 + I2C_IRQENABLE_OFFSET));
			
			//iowrite32 (raddr, Host_Controller.i2c2 + I2C_DATA_OFFSET); 			
			//iowrite32 (data, Host_Controller.i2c2 + I2C_DATA_OFFSET);
			printk(KERN_ALERT "STT = 1");
			iowrite32 (0x8601, Host_Controller.i2c2 + I2C2_CON_OFFSET );
			
			wait_event_interruptible(wait_queue,xrdy_flag == 1);
			printk("El xrdy flag es: %d", xrdy_flag);
			printk("El ardy flag es: %d",ardy_flag);
			iowrite32(raddr,Host_Controller.i2c2 + I2C_DATA_OFFSET);
			iowrite32(data,Host_Controller.i2c2 + I2C_DATA_OFFSET );
			//iowrite32 (0x8602, Host_Controller.i2c2 + I2C2_CON_OFFSET ); //--> Habilitamos y configuramos el modo de funcionamiento dle I2C //8600h = 34304d
		   //printk(KERN_ALERT "Pongo en 1 el I2C2_CON.STT");
	   }
			//iowrite32 (0x8610, Host_Controller.i2c2 + I2C2_CON_OFFSET );	
			iowrite32 (0x8600, Host_Controller.i2c2 + I2C2_CON_OFFSET );	
	return 0;
}

/*
	Lee el valor del registro raddr pasado como argumento y devuelve el resultado.
	Secuencia de read: 
		STT - SA+W - RADDR ; STT - SA+R 
		
*/

static uint8_t i2c_read(int raddr)
{
	uint8_t data = 0;
	
	i2c_bb = ioread32(Host_Controller.i2c2 + I2C_IRQSTATUS_RAW);
	i2c_bb = i2c_bb & 0x8000;
	i2c_bb = i2c_bb >> 11;
	
	ardy_flag = 0;
	if( i2c_bb == 0) //si el BB está en 0
	{
		   
			//configuro DCOUNT
			printk(KERN_ALERT "Configuro DCOUNT");
			iowrite32 (0x01, Host_Controller.i2c2 + I2C_CNT_OFFSET);
		   
			//configuro el valor umbral del BUFFER. 0x00 = 1 , 0x01 = 2, etc
			printk(KERN_ALERT "Configuro BUFFER I2C: Hago CLR del buffer antes de comenzar los envíos de read");
			//iowrite32 (0x40, Host_Controller.i2c2 + I2C_BUFFER_OFFSET);
			
			//reseteo fifo de RX y TX
			i2c_buf = ioread32(Host_Controller.i2c2 + I2C_BUFFER_OFFSET);
			i2c_buf = i2c_buf | 0x40; //TXFIFO RST
			i2c_buf = i2c_buf | 0x4000; //RXFIFO RST
			iowrite32(i2c_buf, Host_Controller.i2c2 + I2C_BUFFER_OFFSET);
			printk(KERN_ALERT "Llego a escribir I2C2 - I2C2 BUFFER %d\n",ioread32(Host_Controller.i2c2 + I2C_BUFFER_OFFSET));
			printk(KERN_ALERT "Cargo DATA");
			
			iowrite32 (raddr, Host_Controller.i2c2 + I2C_DATA_OFFSET); 			
			
			printk(KERN_ALERT "Llego a escribir I2C2 - I2C2 DATA %d\n",ioread32(Host_Controller.i2c2 + I2C_DATA_OFFSET));
		   
			i2c_bb = ioread32(Host_Controller.i2c2 + I2C_IRQSTATUS_RAW);
			i2c_bb = i2c_bb & 0x1000;
			i2c_bb = i2c_bb >> 12;
	   
			printk(KERN_ALERT "%x",i2c_bb);
		   
			printk(KERN_ALERT "Habilito ISR");
			iowrite32 (0x1E, Host_Controller.i2c2 + I2C_IRQENABLE_OFFSET ); //--> Habilitamos y configuramos el modo de funcionamiento del I2C. Habilito interrupción por NOTACK
			printk(KERN_ALERT "Llego a escribir I2C2 - IRQ I2C2 %d\n",ioread32(Host_Controller.i2c2 + I2C_IRQENABLE_OFFSET));
			
			if(ardy_flag)
			{
				printk(KERN_ALERT "ARDY en 1 antes del START:");
			}
			
	
			
			printk(KERN_ALERT "STT = 1");
			iowrite32 (0x8601, Host_Controller.i2c2 + I2C2_CON_OFFSET );
						
			//terminé el primer envío
			

			
			i2c_con = 0;
			i2c_con = ioread32(Host_Controller.i2c2 + I2C2_CON_OFFSET);
			i2c_con = i2c_con & 0xFFFFFDFF; //pongo a cero el bit Tx
			printk(KERN_ALERT "STT en MASTER RX = 1");
			iowrite32 (i2c_con, Host_Controller.i2c2 + I2C2_CON_OFFSET );
			//iowrite32 (0x40,Host_Controller.i2c2 + I2C_BUFFER_OFFSET );
			i2c_buf = ioread32(Host_Controller.i2c2 + I2C_BUFFER_OFFSET);
			//i2c_buf = i2c_buf | 0x40; //TXFIFO RST
			i2c_buf = i2c_buf | 0x4000; //RXFIFO RST
			//data =	ioread32 (Host_Controller.i2c2 + I2C_DATA_OFFSET); 
			i2c_con = ioread32(Host_Controller.i2c2 + I2C2_CON_OFFSET);
			
			//cola de espera: el I2C es mucho más lento que la CPU ==> pongo el proceso a dormir para no bloquear al procesador. 
			wait_event_interruptible(wait_queue, rrdy_flag == 1);
			//wake_up_interruptible(&wait_queue);
			
			printk("El nack flag es: %d", nack_flag);
			printk("El rrdy flag es: %d", rrdy_flag);
			printk("El i2c_irqstatus es: %d",i2c_irq_status);

			data = ioread32(Host_Controller.i2c2 + I2C_DATA_OFFSET);
			nack_flag = 0;
			rrdy_flag = 0;
			printk(KERN_ALERT "IRQ STAT RAW: %x\n",ioread32(Host_Controller.i2c2 + I2C_IRQSTATUS_RAW));
			
			
			printk("ARDY_FLAG: %d",ardy_flag);
			if(ardy_flag)
			{
				printk(KERN_ALERT "ARDY en 1 antes del START:");
				ardy_flag = 0;
			}
				
			printk(KERN_ALERT "i2c_con = %d",i2c_con);

	   }
		//iowrite32 (0x8610, Host_Controller.i2c2 + I2C2_CON_OFFSET );	
		//	iowrite32 (0x8600, Host_Controller.i2c2 + I2C2_CON_OFFSET );	

//	i2c_bb = ioread32(Host_Controller.i2c2 + I2C_IRQSTATUS_RAW);
//	i2c_bb = i2c_bb & 0x8000;
//	i2c_bb = i2c_bb >> 11;


//	if( i2c_bb == 0 ) //si el BB está en 0 y se produjo una condición de STP
//	{
//		   
//			//configuro DCOUNT
//			printk(KERN_ALERT "Configuro DCOUNT");
//			iowrite32 (0x01, Host_Controller.i2c2 + I2C_CNT_OFFSET);
//		   
//			//configuro el valor umbral del BUFFER. 0x00 = 1 , 0x01 = 2, etc
//			printk(KERN_ALERT "Configuro BUFFER I2C");
//			iowrite32 (0x00, Host_Controller.i2c2 + I2C_BUFFER_OFFSET);
//		   		
//			printk(KERN_ALERT "Llego a escribir I2C2 - I2C2 DATA %d\n",ioread32(Host_Controller.i2c2 + I2C_DATA_OFFSET));
//		   
//			i2c_bb = ioread32(Host_Controller.i2c2 + I2C_IRQSTATUS_RAW);
//			i2c_bb = i2c_bb & 0x1000;
//			i2c_bb = i2c_bb >> 12;
//	   
//			printk(KERN_ALERT "%x",i2c_bb);
//		   
//			printk(KERN_ALERT "Habilito ISR");
//			iowrite32 (0x10, Host_Controller.i2c2 + I2C_IRQENABLE_OFFSET ); //--> Habilitamos y configuramos el modo de funcionamiento del I2C
//			printk(KERN_ALERT "Llego a escribir I2C2 - IRQ I2C2 %d\n",ioread32(Host_Controller.i2c2 + I2C_IRQENABLE_OFFSET));
//			
//			i2c_con = 0;
//			i2c_con = ioread32(Host_Controller.i2c2 + I2C2_CON_OFFSET);
//			i2c_con = i2c_con & 0xFFFFFDFF; //pongo a cero el bit Tx
//			printk(KERN_ALERT "STT = 1");
//			iowrite32 (i2c_con, Host_Controller.i2c2 + I2C2_CON_OFFSET );
//			data =	ioread32 (Host_Controller.i2c2 + I2C_DATA_OFFSET); 
//		   //iowrite32 (0x8601, Host_Controller.i2c2 + I2C2_CON_OFFSET ); //--> Habilitamos y configuramos el modo de funcionamiento dle I2C //8600h = 34304d
//		   //printk(KERN_ALERT "Pongo en 1 el I2C2_CON.STT");
//	}
			
			//iowrite32 (0x8410, Host_Controller.i2c2 + I2C2_CON_OFFSET );	
			iowrite32 (0x8402, Host_Controller.i2c2 + I2C2_CON_OFFSET );	
			iowrite32 (0x8600, Host_Controller.i2c2 + I2C2_CON_OFFSET );	

			
			
	return i2c_data;
}



/*
	Función que utilizamos para atender la interrupción virtual que solicitamos en el probe
*/
static irqreturn_t Mi_handler_interrupt(int irq){
    
	i2c_irq_status = ioread32(Host_Controller.i2c2 + I2C_IRQSTATUS_RAW);
	i2c_xrdy = i2c_irq_status;
	i2c_ardy = i2c_irq_status;
	i2c_nack = i2c_irq_status;
	i2c_rrdy = i2c_irq_status;;
	//reviso si el bit de XRDY está en 1, lo cual significa que se produjo la interrupción de Tx
	i2c_xrdy = i2c_xrdy & 0x10;
	i2c_xrdy = i2c_xrdy >> 4;
	
	if(i2c_xrdy)
	{
		//si entro acá es porque hubo una interrupción por XRDY => tengo que limpiar el bit utilizando el I2C_IRQENABLE_CLR	
		iowrite32(0x10,Host_Controller.i2c2 + I2C_IRQSTATUS);
		xrdy_flag = 1;
		wake_up_interruptible(&wait_queue);
	}
	
	
	//reviso si el bit de ARDY está en 1, lo cual significa que se produjo una condición de STP
	//i2c_ardy = ioread32(Host_Controller.i2c2 + I2C_IRQSTATUS_RAW);
	i2c_ardy = i2c_ardy & 0x4;
	i2c_ardy = i2c_ardy >> 2;	
	
	if(i2c_ardy)
	{
		//si entro acá es porque hubo una interrupción por ARDY => tengo que limpiar el bit utilizando el I2C_IRQENABLE_CLR	
		iowrite32(0x4,Host_Controller.i2c2 + I2C_IRQSTATUS);
		ardy_flag = 1;
	}
	
	//i2c_nack = ioread32(Host_Controller.i2c2 + I2C_IRQSTATUS_RAW);
	i2c_nack = i2c_nack & 0x02;
	i2c_nack = i2c_nack >> 1;	
	
	if(i2c_nack)
	{
		//si entro acá es porque hubo una interrupción por NACK => tengo que limpiar el bit utilizando el I2C_IRQENABLE_CLR	
		iowrite32(0x02,Host_Controller.i2c2 + I2C_IRQSTATUS);
		nack_flag = 1;
		//wake_up_interruptible(&wait_queue);
	}

	i2c_rrdy = i2c_rrdy & 0x08 ;
	i2c_rrdy = i2c_rrdy >> 3 ;

	if(i2c_rrdy)
	{

		//si entro acá es porque hubo una interrupción por RRDY => siginifica que está lista la información en la FIFO del Sitara para ser leída
		iowrite32(0x08,Host_Controller.i2c2 + I2C_IRQSTATUS);
		i2c_data = ioread32(Host_Controller.i2c2 + I2C_DATA_OFFSET);
		rrdy_flag = 1;
		wake_up_interruptible(&wait_queue);
		//iowrite32(0x08, Host_Controller.i2c2 + I2C_IRQENABLE_CLR);
	}	
	
	//limpio los otros flags de interupcion  que se podrán haber levantado, pero que no representan condiciones que me interesen.
	//bits 31-15: RSV. Bit 12: la escritura no tiene efecto => escribo 0
	iowrite32(0x6FFF,Host_Controller.i2c2 + I2C_IRQSTATUS);	
    return IRQ_HANDLED;
}

/*
	Al desinstalar nuestro driver, debemos remover nuestro platform_driver
*/
static int funcion_remove(struct platform_device * pdev){    
	printk(KERN_ALERT "\n  Entro a REMOVE \n");
	iowrite32(0x02,Host_Controller.i2c2 + 0x10);
	free_irq(Host_Controller.irq,NULL);
	iounmap(Host_Controller.cm_per);
	iounmap(Host_Controller.i2c2);
	iounmap(Host_Controller.mode);
	
	return 0;
}


/*
	Función que utilizará se utilizará en espacio usuario para leer los valores del dispositivo.
	
	*file: puntero a archivo (abrimos el dispositivo \dev\mpu como un archivo
	*buf: buffer donde escribiremos los datos leídos, para que el usuario pueda tomarlos desde allí
	count: tamaño de la transferencia de datos
	nose: ptr a un objeto del tipo "log offset type", que indica la posición del archivo a la que está accediendo el usuario
*/
static ssize_t mpu_read(struct file *file, char *buf, size_t count, loff_t *nose)
{
	//tengo que trabajar con un buffer de kernel, cargar los datos allí y luego usar la función que el kernel provee para pasar datos del buffer de kernel al del usuario
	
	uint8_t *buffer;
	int i = 0;
	char fifo_en = 0;
	unsigned long ctu = 0;
	
	buffer = kmalloc(sizeof(uint8_t)*paq_size*count,GFP_KERNEL); //cada paquete tiene un tamaño de 14 bytes => sizeof(uitn8_t) = 1byte * cantidad de paquetes * bytes por paquetetes
	if(buffer == NULL)
	{	
		printk(KERN_ALERT "*** ERROR AL ALLOCAR PARA LECTURA ***");
		return -EFAULT;
	}
	
if(fifo_en == 0)	
{
	for(i = 0; i < (count*paq_size); i+=paq_size)
	{
	
		//leo los registros correpondientes a la lectura del eje X del acelerómetro
//		AXH = i2c_read(MPU_ACC_XH_OFFSET);
//		AXH = i2c_read(MPU_ACC_XH_OFFSET);
//		printk(KERN_ALERT "XH: = %x",AXH);
//		AXL = i2c_read(MPU_ACC_XL_OFFSET);
//		AXL = i2c_read(MPU_ACC_XL_OFFSET);
//		printk(KERN_ALERT "XL: = %x",AXL);
//	
//		//leo los registros correpondientes a la lectura del eje Y del acelerómetro
//		i2c_read(MPU_ACC_YH_OFFSET);
//		AYH = i2c_read(MPU_ACC_YH_OFFSET);
//		printk(KERN_ALERT "YH: = %x",AYH);
//		i2c_read(MPU_ACC_YL_OFFSET);
//		AYL = i2c_read(MPU_ACC_YL_OFFSET);
//		printk(KERN_ALERT "YL: = %x",AYL);
//	
//		//leo los registros correpondientes a la lectura del eje Z del acelerómetro
//		AZH =i2c_read(MPU_ACC_ZH_OFFSET);
//		AZH = i2c_read(MPU_ACC_ZH_OFFSET);
//		printk(KERN_ALERT "ZH: = %x",AZH);
//		AZL = i2c_read(MPU_ACC_ZL_OFFSET);
//		AZL = i2c_read(MPU_ACC_ZL_OFFSET);
//		printk(KERN_ALERT "ZL: = %x",AZL);
	
	
//		AX = (uint16_t) (AXH << 8) + (uint16_t) AXL;
//		AY = (uint16_t) (AYH << 8) + (uint16_t) AYL;
//		AZ = (uint16_t) (AZH << 8) + (uint16_t) AZL;
//		
//		printk("Llegué a AZ HEX: %x",AZ);
//		AXF = AX*100/16384;
//		AX = AXF/100;
//		AXF = AXF - AX*100;
//		
//		AYF = AY*100/16384;
//		AY = AYF/100;
//		AYF = AYF - AY*100;
//		
//		AZF = AZ*100/16384;
//		AZ = AZF/100;
//		AZF = AZF - AZ*100;
		
//		printk(KERN_ALERT "AZ: %d",AZ);
//		printk(KERN_ALERT "Pasé los cálculos");
		i2c_read(MPU_ACC_XH_OFFSET);
		buffer[i] =   i2c_read(MPU_ACC_XH_OFFSET);
		i2c_read(MPU_ACC_XL_OFFSET);
		buffer[i+1] =  i2c_read(MPU_ACC_XL_OFFSET);
		i2c_read(MPU_ACC_YH_OFFSET);
		buffer[i+2] =  i2c_read(MPU_ACC_YH_OFFSET);
		i2c_read(MPU_ACC_YL_OFFSET);
		buffer[i+3] =  i2c_read(MPU_ACC_YL_OFFSET);
		i2c_read(MPU_ACC_ZH_OFFSET);
		buffer[i+4] =  i2c_read(MPU_ACC_ZH_OFFSET);
		i2c_read(MPU_ACC_ZL_OFFSET);
		buffer[i+5] =  i2c_read(MPU_ACC_ZL_OFFSET);

//		buffer[i] =    AX;
//		buffer[i+1] =  AXF;
//		buffer[i+2] =  AY;		printk(KERN_ALERT "buffer i+4: %x",buffer[i+4]);
//		buffer[i+3] =  AYF;		printk(KERN_ALERT "buffer i+5: %x",buffer[i+5]);
//		buffer[i+4] =  AZ;		printk(KERN_ALERT "Ahora... a rezar"); 
//		buffer[i+5] =  AZF;
	}
}	

	
	if(FIFOC > 0 && fifo_en == 1)
	{
		
	//para saber la cantidad de bytes en la FIFO: 1ra lectura de FIFOCH, actualiza el valor. 
		i2c_read(MPU_FIFOCH_OFFSET); 
		FIFOCH = i2c_read(MPU_FIFOCH_OFFSET);
		FIFOCL = i2c_read(MPU_FIFOCL_OFFSET);
		
		FIFOC = (uint16_t) (FIFOCH << 8) + (uint16_t) FIFOCL;
		printk(KERN_ALERT "Cant de bytes en la FIFO = %d", FIFOC);
	
	/*
		Si hay datos en la FIFO => leo. 
		Orden de escritura en la FIFO: por nro de registro en orden descendente, de 59 a 96
		59 a 64: Acelerómetro => 59-60 = X ; 61-62 = Y ; 63-64 = Z 
		65 a 66: temperature
		67 a 72: Gyro
			
	*/
	
		
		i = 0;
		
		for(i = 0; i < (count*paq_size); i+=paq_size)
		{

			//reseteo la FIFO antes de leer
			i2c_send(MPU_USRCTRL_OFFSET,0x44);
			
			//leo los registros correpondientes a la lectura del eje X del acelerómetro
			buffer[i]  = i2c_read(MPU_FIFORW_OFFSET);	
			buffer[i+1]  = i2c_read(MPU_FIFORW_OFFSET);

			//leo los registros correpondientes a la lectura del eje Y del acelerómetro
			buffer[i+2] = i2c_read(MPU_FIFORW_OFFSET);
			buffer[i+3] = i2c_read(MPU_FIFORW_OFFSET);
			
			
			//leo los registros correpondientes a la lectura del eje Z del acelerómetro
			buffer[i+4] = i2c_read(MPU_FIFORW_OFFSET);
			buffer[i+5] = i2c_read(MPU_FIFORW_OFFSET);
			
			
		}
		

	//	AZ = (AZ)/(16384);
		
	}
	
	
	
	if( (ctu = __copy_to_user(buf, buffer, (count*paq_size))) < 0)
	{
		return -EFAULT;
	}

	
	
	

	kfree(buffer);
	return count - ctu;
}


//wait_event_interruptible
	
/*
	No realizamos lecturas al dispositivo
*/	
static ssize_t mpu_write(struct file *file, const char *buf, size_t count, loff_t *nose)
{
	return 0;
}
	

/*
	Función Open para abrir el dispositivo. 
	La primera operación que se realiza sobre el dispositivo, sin embargo el driver no está obligado a declarar un método. Si el driver no declara un método de open(), al realizar una llamada a esta función desde un programa
	usuario, la misma siempre será exitosa. 
	
	Open debe realizar las siguientes operaciones: 
		1) Check for device-specific errors (such as device-not-ready or similar hardware problems)
		2) initialize the device if it is being opened for the first time
		3) Update the f_op pointer, if necessary
		4) Allocate and fill any data structure to be put in filp->private_data
		
*/	
static ssize_t mpu_open(struct inode *inode, struct file * file)
{
	//primero: definir qué dispositivo estamos intentando abrir. La estructura "inode" contiene esta información en el campo i_cdev 
	
	//verificar que el dispositivo de HW esté: leo el WHO AM I? Y si no da 0x68 => no está conectado el sensor
	if ( (i2c_read(MPU_WHOAMI_OFFSET) != 0x68))
	{
		printk(KERN_ALERT "*** No se encontró el dispositivo de HW");
		return -1;
	}	
	
	    //I2C clock domain configuration

		// página 1253 manual Sitara 335x. El offset 44h dentro del Periférico CM_PER, corresponde al clock del I2C2- 
		//	 Bits 1-0 = MODULEMODE = 2h (module explicity enabled) 
		//	 Bits 17-16 = IDLEST = 0h (Module is fully functional) 
		
		// => en el offset 44h, escrubumos un 0x02 => IDLEST = 0h y MODULEMODE = 2h
        iowrite32 (0x02, Host_Controller.cm_per + CM_PER_I2C2_CLK_OFFSET); //--> estamos habiliando el clock del I2C2
        printk(KERN_ALERT "Llego a escribir CMPER - I2C2_CLKCTR %d\n",ioread32(Host_Controller.cm_per + CM_PER_I2C2_CLK_OFFSET));
        printk(KERN_ALERT "Ya escribí CM_PER y habilité el clock del I2C2 \n");
        
   
		/*
			Control module: seleccionamos la funcionalidad I2C para los pines 19 y 20 del P9
				
			SDA offset 978h: 
							bit 31-7: rsv
							bit 6: slewrate (0 -> fast)
							bit 5: rxactive (1 -> rx_enabled)
							bit 4: Pullup or down (1 -> up)
							bit 3: Pullup/down enable (0 -> enabled)
							bit 2-0: 011 (function 3 = I2C SDA)
							
								
			SCL offset 97Ch: 
		
		*/
		iowrite32 (0x33, Host_Controller.mode + I2C2_SDA_OFFSET); //--> elejimos el modo del pin
		printk(KERN_ALERT "Llego a escribir CONTROL MODULE - SDA %d\n",ioread32(Host_Controller.mode + I2C2_SDA_OFFSET));
		iowrite32 (0x33, Host_Controller.mode + I2C2_SCL_OFFSET); //--> elejimos el modo del pin
		printk(KERN_ALERT "Llego a escribir CONTROL MODULE - SCL %d\n",ioread32(Host_Controller.mode + I2C2_SCL_OFFSET));
		
		
		printk(KERN_ALERT "Ya escribí elegí el modo I2C2 para los pines 19 y 20 de P9 \n");
		
        //i2c2
		/*
			Así como habilitamos el clock, ahora tenemos que configuar el modulo en sí. 
			
			
			Del Technical reference del Sitara (p. 4599)
			
			Module Configuration Before Enabling the Module
				1. Program the prescaler to obtain an approximately 12-MHz I2C module clock (I2C_PSC = x; this value
				is to be calculated and is dependent on the System clock frequency).
				2. Program the I2C clock to obtain 100 Kbps or 400 Kbps (SCLL = x and SCLH = x; these values are to
				be calculated and are dependent on the System clock frequency).
				3. Configure its own address (I2C_OA = x) - only in case of I2C operating mode (F/S mode).
				4. Take the I2C module out of reset (I2C_CON:I2C_EN = 1)
				
			Initialization Procedure
				1. Configure the I2C mode register (I2C_CON) bits.
				2. Enable interrupt masks (I2C_IRQENABLE_SET), if using interrupt for transmit/receive data.
				3. Enable the DMA (I2C_BUF and I2C_DMA/RX/TX/ENABLE_SET) and program the DMA controller) -
				only in case of I2C operating mode (F/S mode), if using DMA for transmit/receive data
				
				
			Configure Slave Address and DATA Counter Registers
				In master mode, configure the slave address (I2C_SA = x) and the number of byte associated with the
				transfer (I2C_CNT = x)	
			
			
		*/
	

		
		/*
			configuro pre-scaler del I2C2
			
			Se usan los primeros 8 bits, los demás están reservados. 0x0 = divide por 1. 0x1 = divide por 2. ... 0xff = divider por 256	(SCLK / (1+PSC)) 
			
			El clock que reibimos es 192MHz dividio 4 => 48MHz
			
			Freq = SCLK / (1+PSC)

			(1+PSC) = SCLK / Freq
			
			PSC = SCLK / Freq -1 			Freq = 12MHz SCLK = 19.2MHz
			
			PSC = 48 / 12 - 1	
				
			PSC = 3
			
			
			Si SCLK = 48MHz => PSC = 3h
			
			
		*/
        iowrite32 (0x03, Host_Controller.i2c2 + I2C2_PSC_OFFSET); //--> estamos habiliando el clock del I2C2
		printk(KERN_ALERT "Llego a escribir I2C2 - PSC %d\n",ioread32(Host_Controller.i2c2 + I2C2_PSC_OFFSET));
		
		/*
			configuro el duty cycle del I2C configurando SCLL y SCLH
			
			
			
			Se usan los primeros 8 bits, los demás están reservados.  tLOW = (SCLL + 7) * ICLK
			
			
			
			
			Leído					Escrito

SCLL		61h = 0110 0001b		3dh = 0011 1101
SCLH		50h = 0101 0000b		32h = 0011 0010

		*/
        iowrite32 (0x09, Host_Controller.i2c2 + I2C2_SCLL_OFFSET ); //--> estamos habiliando el clock del I2C2		//3Dh = 61d
		printk(KERN_ALERT "Llego a escribir I2C2 - SCLL %d\n",ioread32(Host_Controller.i2c2 + I2C2_SCLL_OFFSET));
		
		/*
			configuro SCLH del I2C2
			
			 Se usan los primeros 8 bits, los demás están reservados.  tHIGH = (SCLH + 5) * ICLK   Donde ICLK = 12MHz
			El registro está partido en dos
			
			
			Para el modo Standar, el duty cycle es del 45,977% (aprox) (tomado de especificación del bus I2C )
			
			tLOW + tHIGH = 1/(100KHz) (standard mode)
			
			
		====>	
			
			I2SCLH & I2SCLL: The values in these two registers will be used to set the data rate of the I2C communication. The bit frequency is given the formula

				Bit Frequency = Fclk / (I2SCLH+I2SCLL)
			
		*/
        iowrite32 (0x08, Host_Controller.i2c2 + I2C2_SCLH_OFFSET ); //--> estamos habiliando el clock del I2C2		//32h = 50d
		printk(KERN_ALERT "Llego a escribir I2C2 - SCLH %d\n",ioread32(Host_Controller.i2c2 + I2C2_SCLH_OFFSET));
		
		
		
		/*
			Tenemos que configurar el slave address
			I2C_SA:
				Bit 31-10: rsv (0)
				Bit 9-0: slave address 0001101000
			
			
			I2C_CNT:
				Bit 31-16: rsv
				Bit 15-0: DCOUNT
		*/
		iowrite32 (0x68, Host_Controller.i2c2 + I2C_SA_OFFSET ); //--> Habilitamos y configuramos el modo de funcionamiento del I2C. Slave address
		
		iowrite32 (0x01, Host_Controller.i2c2 + I2C_CNT_OFFSET); //--> Habilitamos y configuramos el modo de funcionamiento del I2C. Cantidad de datos a transmitir
		
		/*
			Configurar la dirección del master (I2C_OA)
			
			I2C_OA = 1010 0001 = 0xA1
		*/
		
		//iowrite32 (0xA1, Host_Controller.i2c2 + I2C_OA_OFFSET ); //--> Habilitamos y configuramos el modo de funcionamiento del I2C
		 printk(KERN_ALERT "Llego a escribir I2C2 - OA %d\n",ioread32(Host_Controller.i2c2 + I2C_OA_OFFSET));
		/*
			Take the I2C module out of reset
			I2C_CON:I2C_EN = 1
			
			I2C_EN es el bit 15 del registro I2C_CON
			
			Bits 31-16: rsv (0)
			Bit 15: Enable (1)
			Bit 14: rsv (0)
			Bit 13-12: OPMODE (00)
			Bit 11: StartByte (0)
			Bit 10: Master (1)
			Bit 9: TRX (1)
			Bit 8: XSA (0) (MPU 7 bit address)
			Bit 7: XOA0 (0) (MPU 7 bit address)
			Bit 6: XOA1 (0) (MPU 7 bit address)
			Bit 5: XOA2 (0) (MPU 7 bit address)
			Bit 4: XOA3 (0) (MPU 7 bit address)
			Bit 3-2: rsv (00)
			Bit 1: STP (0) (no action or stop condition detected)
			Bit 0: STT (0) (no action or start condition detected)
		*/
		iowrite32 (0x8600, Host_Controller.i2c2 + I2C2_CON_OFFSET ); //--> Habilitamos y configuramos el modo de funcionamiento dle I2C //8600h = 34304d

		
		printk(KERN_ALERT "\n El I2C2_CON es %d", ioread32(Host_Controller.i2c2 + I2C2_CON_OFFSET));
		
		
		/*	
			El I2C_SYSTEST permite forzar el CLK (SCL). Se lo setea en modo "libre" o contínuo para que funcione sin detenerse. Sólo para pruebas
		*/
		
//		iowrite32 (0x8000, Host_Controller.i2c2 + I2C_SYSTEST_OFFSET ); //--> Habilitamos y configuramos el modo de funcionamiento dle I2C //8600h = 34304d
//		printk(KERN_ALERT "\n El I2C2_SYSTESTS es %x", ioread32(Host_Controller.i2c2 + I2C_SYSTEST_OFFSET));
//		
//		
//		i2c_systst = ioread32(Host_Controller.i2c2 + I2C_SYSTEST_OFFSET);
//		i2c_systst = i2c_systst | 0x2000;  
//		iowrite32 (i2c_systst, Host_Controller.i2c2 + I2C_SYSTEST_OFFSET ); //--> Habilitamos y configuramos el modo de funcionamiento dle I2C //8600h = 34304d
//		printk(KERN_ALERT "\n El I2C2_SYSTESTS es %x", ioread32(Host_Controller.i2c2 + I2C_SYSTEST_OFFSET));
		

		
		/*
			Ahora hay que habilitar las interrupciones.
			I2C_IRQENABLE_SET
			
			Bit 31-15: rsv (0)
			Bit 14: (0)
			Bit 13: (0)
			Bit 12: (0)
			Bit 11: (0)
			Bit 10: (0)
			Bit 9: (0)
			Bit 8: BF_IE (1)
			Bit 7: (0) 
			Bit 6: (0)
			Bit 5: (0)
			Bit 4: XRDY_IE (1)
			Bit 3: RRDY_IE (1)
			Bit 2: ARDY_IE (1)
			Bit 1: (0)
			Bit 0: (0)
			
		*/
		//iowrite32 (0x11C, Host_Controller.i2c2 + I2C_IRQENABLE_OFFSET ); //--> Habilitamos y configuramos el modo de funcionamiento dle I2C
        printk(KERN_ALERT "Llego a escribir I2C2 - IRQ I2C2 %d\n",ioread32(Host_Controller.i2c2 + I2C_IRQENABLE_OFFSET));
        
		/*
			Tenemos que configurar el slave address
			I2C_SA:
				Bit 31-10: rsv (0)
				Bit 9-0: slave address 0001101000
			
			
			I2C_CNT:
				Bit 31-16: rsv
				Bit 15-0: DCOUNT
		*/
		//iowrite32 (0x68, Host_Controller.i2c2 + I2C_SA_OFFSET ); 
		
		//iowrite32 (0x00, Host_Controller.i2c2 + I2C_CNT_OFFSET); 
		
	
	
		

/******************************** CONFIGURACION MPU6050*************************************/		

	//el sensor amanece en sleep (0x040). Lo escribo en 0x00 para sacarlo de ese modo.
	i2c_send(MPU_PWRMGMT_OFFSET,0x00);
	/*
		configura el pin externo FSYNC (no utilizado) y el DLPF que se aplicará a los datos del acelerómetro y giróscopo
		
		Bit 7: -
		Bit 6: -
		Bit 5 - 3: EXT_SYNC_SET (0)
		Bit 2 - 0: DLPF 

	*/
	i2c_send(MPU_CONFIG_OFFSET,0x06); 
	
	/*
		configura el giroscopio: rango y self_test 
		
		Bit 7: XG_ST
		Bit 6: YG_ST
		Bit 5: ZG_ST
		Bit 4 - 3: FS_SEL
		Bit 2 - 0: RSV 

	*/
	i2c_send(MPU_GYR_CONFIG_OFFSET,0x0);
	
	/*
		configura el giroscopio: rango y self_test 
		
		Bit 7: XG_ST
		Bit 6: YG_ST
		Bit 5: ZG_ST
		Bit 4 - 3: FS_SEL
		Bit 2 - 0: RSV 

	*/
	i2c_send(MPU_ACC_CONFIG_OFFSET,0x00); //0x20 habilita el ZA_TEST 
	
	
	
	/*
		permite habilitad la FIFO, i2c master mode y la interfaz i2c primaria.
		Bit 7: rsv 				(0)
		Bit 6: FIFO_EN 			(1)
		Bit 5: I2C_MST_EN 		(0)
		Bit 4: I2C_IF_DIS 		(0)
		Bit 3: rsv 				(0)	
		Bit 2: FIFO_RESET 		(0)
		Bit 1: I2C_MST_RESTET 	(0)		
		Bit 0: SIG_COND_RST 	(0)
	*/	
	i2c_send(MPU_USRCTRL_OFFSET,0x40);
	printk(KERN_ALERT "EL USRCTRL es: %x",i2c_read(MPU_USRCTRL_OFFSET));
	
	/*
		Habilita el uso de la FIFO interna (1024 bytes) para cada sensor para el que se lo habilite
		Bit 7: TEMP 	(1)
		Bit 6: XG 		(1)
		Bit 5: YG 		(1)
		Bit 4: ZG 		(1)
		Bit 3: ACCEL 	(1)	
		Bit 2: SLV2 	(0)
		Bit 1: SLV1 	(0)		
		Bit 0: SLV0 	(0)
	*/
	i2c_send(MPU_FIFO_EN_OFFSET,0x08); //--> sólo acelerómetro´
	if(ardy_flag)
	{
	   ardy_flag = 0;
		printk(KERN_ALERT "limpié ardy flag después de habiltiar FIFO!");
	}
//	printk(KERN_ALERT "Leí del MPU %d",i2c_read(MPU_FIFO_EN_OFFSET));
		//pongo en 1 el bit de Start
	

	//se retorna 0. El file descriptor que recibe el usuario es manejado internamente por el kernel
	return 0; 
}

/*
	Función release para cerrar el dispositivo (close). 
	Al igual que open, no es necesario que el driver la defina
*/	
static ssize_t mpu_release(struct inode *inode, struct file * file)
{
	
		/*
		Take the I2C module out of reset
		I2C_CON:I2C_EN = 1
		
		I2C_EN es el bit 15 del registro I2C_CON
		
		Bits 31-16: rsv (0)
		Bit 15: Enable (1)
		Bit 14: rsv (0)
		Bit 13-12: OPMODE (00)
		Bit 11: StartByte (0)
		Bit 10: Master (1)
		Bit 9: TRX (1)
		Bit 8: XSA (0) (MPU 7 bit address)
		Bit 7: XOA0 (0) (MPU 7 bit address)
		Bit 6: XOA1 (0) (MPU 7 bit address)
		Bit 5: XOA2 (0) (MPU 7 bit address)
		Bit 4: XOA3 (0) (MPU 7 bit address)
		Bit 3-2: rsv (00)
		Bit 1: STP (0) (no action or stop condition detected)
		Bit 0: STT (0) (no action or start condition detected)
	*/
	iowrite32 (0x0000, Host_Controller.i2c2 + I2C2_CON_OFFSET ); //--> Deshabilito el I2C2
	
	
	
	//Dejo de darle clock al módulo I2C2
	// página 1253 manual Sitara 335x. El offset 44h dentro del Periférico CM_PER, corresponde al clock del I2C2- 
	//	 Bits 1-0 = MODULEMODE = 2h (module explicity enabled) 
	//	 Bits 17-16 = IDLEST = 0h (Module is fully functional) 
	
	// => en el offset 44h, escrubumos un 0x02 => IDLEST = 0h y MODULEMODE = 2h
	iowrite32 (0x02, Host_Controller.cm_per + CM_PER_I2C2_CLK_OFFSET); //--> estamos habiliando el clock del I2C2
	printk(KERN_ALERT "Llego a escribir CMPER - I2C2_CLKCTR %d\n",ioread32(Host_Controller.cm_per + CM_PER_I2C2_CLK_OFFSET));
	printk(KERN_ALERT "Ya escribí CM_PER y habilité el clock del I2C2 \n");
	
	
	
	//regreso el sensor al modo sleep(0x040). 
	i2c_send(MPU_PWRMGMT_OFFSET,0x40);
	
	//Apago el clock del I2C2
	
	
	return 0;
}

//defino las operaciones que pueden realizarse sobre el "archivo"
/*
	Las operaciones que se pueden realizar sobre archivos, dependen de los drivers que manejan esos archivos. Esas operaciones están definidas como instancias de "struct file operations".  
	Esta estructura contiene un juego de callbacks, que manejaran cualquier syscall realizada en espacio de usuario sobre un archivo. 	
*/
struct file_operations mpu_fops =
{
  read: mpu_read,
  write: mpu_write,      
  open: mpu_open,
  release: mpu_release
};

/*los char devices, se representan en el kernel como instancias de "struct cdev":
struct cdev{
					struct kobject kogb;
					struct module *owner;
					const struct file_operations *ops;
					struct list_head list;
					dev_t dev;
					unsigned int count;
				};*/
static struct cdev mpu_cdev;


//????
static int my_dev_uevent(struct device *dev, struct kobj_uevent_env *env)
{
  add_uevent_var(env, "DEVMODE=%#o", 0666);
  return 0;
}


//rutina de istalación del driver
static int mpu_init( void )
{
  
  
  /*
		Allocating and registering a char device
		
						1. Reservar un nro major y un rango de nros menores usando "alloc_chrdev_region"
						2. Crear una clase para nuestro dispositivo utilizando "class_create()", visible en /sys/class/
						3. Setear una struct file_operation (para darsela a cdev_init), y por cada dispositivo que deseemos crear llamar a cdev_init() y cdev_add() para registrar el dispositivo
						4. Hacer un device_create() para cada dispositivo. Esto resultará en nuestro dispositivo siendo creado en /dev.
  */
  
  MODULE_LICENSE("GPL"); //licencia abierta GPL
  
	/*
		@fn: int alloc_chrdev_region(dev_t *dev, unsigned int firstminor,unsigned int count, char *name)
		@brief: Registramos y allocamos un par de nros mayor y menor
		@params: 
				dev: alojará el primer nro en el rango de nrs solicitados
				firstminor: primer nro menor a utilizar (0 por lo gral)
				cout: cantidad de nros contiguos de device numbers que queremos solicitar
				name: nombre que se asociará al dispositivo
	*/
	
  if (alloc_chrdev_region( &dev, 0, 1, "mpu" ) < 0)	//int alloc_chrdev_region(dev_t *dev, unsigned int firstminor, unsigned int count, char *name);
  {  
    printk( KERN_ALERT "No se puede ubicar la region\n" );
    return -1;
  }
  
  //creamos una clase para nuestro driver
  cl = class_create( THIS_MODULE, "chardev" );
  if ( cl == NULL )	//class create devuelve NULL => error
  {
    printk( KERN_ALERT "No se puede crear la clase\n" );
    // Borrar lo asignado para no tener memory leak en kernel
    unregister_chrdev_region( dev, 1 );	
	cdev_del(&mpu_cdev);
    return -1;
  }
  
  // Asignar el callback que pone los permisos en /dev/mpu
  cl -> dev_uevent = my_dev_uevent;
  //Hacer un device_create() para cada dispositivo. Esto resultará en nuestro dispositivo siendo creado en /dev.
  if( device_create( cl, NULL, dev, NULL, "mpu" ) == NULL )
  {
    printk( KERN_ALERT "No se puede crear el device driver\n" );
    // Borrar lo asignado para no tener memory leak en kernel
    class_destroy(cl);
    unregister_chrdev_region( dev, 1 );
	cdev_del(&mpu_cdev);
    return -1;
  }
  //
  cdev_init(&mpu_cdev, &mpu_fops); //mpu_cdev: nuestra struct cdev . mpu_fops: es nuestro file operations (donde definimos qué operaciones podemos realizar sobre nuestro char device)
  mpu_cdev.owner = THIS_MODULE;
  mpu_cdev.ops = &mpu_fops;
  
  //por cada dispo que deseemos crear hay que llamar a cdev_init y cdev_add
  if (cdev_add(&mpu_cdev, dev, 1) == -1)
  {
    printk( KERN_ALERT "No se pudo agregar el device driver al kernel\n" );
    // Borrar lo asignado para no tener memory leak en kernel
    device_destroy( cl, dev );
    class_destroy( cl );
    unregister_chrdev_region( dev, 1 );
	cdev_del(&mpu_cdev);
    return -1;
  }
  
  
    /*
		En nuestro caso estamos trabajando con un dispositivo I2C, el cual no puede enumerarse (como sí lo hacen los dispositivos USB). Por ello, debemos escribir un platform driver, que manejará el "bus virtual" en el que se monta
			nuestro dispositivo. Entonces, en la función de instalación de nuestro driver, debemos instanciar nuestro platform_driver
  */
 
	if(platform_driver_register(&Mi_I2C_Host_Controller)<0) //con esta funcion registramos dentro de nuestro modulo, al platform_driver
	{
		printk(KERN_ALERT "Error al registar el platform driver\n"); 
		printk(KERN_ALERT "Error al registar el platform driver\n"); 
		device_destroy( cl, dev );
		class_destroy( cl );
		unregister_chrdev_region( dev, 1 );
		cdev_del(&mpu_cdev);
	}
  
  printk(KERN_ALERT "Se registro el platfom driver!\n"); 

 
  printk(KERN_ALERT "Driver I2C para MPU6050 instalado con numero mayor %d y numero menor %d\n", MAJOR(dev), MINOR(dev)); // MAJOR y MINOR extraen los nros mayor y menor de la estrcutura cdev
  return 0;
  
  

}

static void mpu_exit( void )
{
  printk(KERN_ALERT "Desinstalo el Driver!\n");
  platform_driver_unregister(&Mi_I2C_Host_Controller);    //llama a funcion remove 	
  
  // Borrar lo asignado para no tener memory leak en kernel
  cdev_del(&mpu_cdev);   //---->To remove a char device from the system, call:   void cdev_del(struct cdev *dev);    LINUX DEVICE DRIVERS 3ED Page 56. Esta función debería liberar la memoria alojada para la estructura cdev
 // kfree(&mpu_cdev);
  device_destroy( cl, dev );
  class_destroy( cl );
  unregister_chrdev_region(dev, 1);
  

  printk(KERN_ALERT "Driver MPU desinstalado.\n");
}

module_init(mpu_init);
module_exit(mpu_exit);



/*
	para hacer el read y el write: configurar el clock, hacer el iomap para acceder a memoria, energizar el 
*/

