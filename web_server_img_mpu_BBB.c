/**********************************************************/
/* Web server que lee datos del MPU6050             	  */
/*                                                        */
/* Headers HTTP relevantes del cliente:                   */
/* GET {path} HTTP/1.x                                    */
/*                                                        */
/* Encabezados HTTP del servidor enviados como respuesta: */
/* HTTP/1.1 200 OK                                        */
/* Content-Length: nn (longitud del HTML)                 */
/* Content-Type: text/html; charset=utf-8                 */
/* Connection: Closed                                     */
/*                                                        */
/* Después de los encabezados va una línea en blanco y    */
/* luego el código HTML.                                  */
/*                                                        */
/* 							  */
/* 	Author: por Ian Sztenberg (09/2022)		  */
/* 	modified: 02-12-2022		                  */
/**********************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <sys/ipc.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <signal.h>
#include <math.h> 

#define MAX_CONN 10 //Nro maximo de conexiones en espera

#define paq_size 6

#define OKMETHOD    1
#define BADMETHOD   2
#define BADRESRC    3
#define GRAPH       4

#define MAX_LINE_SZ	250

//tenmos 2 registros de 8bits por eje. Y tenemos 3 ejes => 1 eje = 16its Por lo tanto 3 ejes = 48bits = 6bytes => Acc + Gyro + Temp = 6bytes + 6bytes + 2bytes = 14bytes
uint8_t *buffer;

uint8_t AXL = 0;
uint8_t AXH = 0;
uint8_t AYL = 0;
uint8_t AYH = 0;
uint8_t AZL = 0;
uint8_t AZH = 0;

int16_t *AZ;
int16_t *AY;
int16_t *AX;
float *AXF; 
float *AYF; 
float *AZF; 

//variables de archivo de cfg.
FILE *fconfig;
int refresh = 2;
int cant_con;
int backlog;
int win_sz;
//double coefs[10];
//double coefs2[10];
//inicializo con los valores default. Si existe el archivo de cfg => se reemplazarán por los que correspondan
double coefs[11] =  {-0.05050399753520547,-0.030696657906964218,0.03275526742496901,0.14641971643381654,0.2585277223282103,0.30544025095809957,0.2585277223282103,0.14641971643381654,0.03275526742496901,-0.030696657906964218,-0.05050399753520547};
double coefs2[11] = {-0.05050399753520547,-0.030696657906964218,0.03275526742496901,0.14641971643381654,0.2585277223282103,0.30544025095809957,0.2585277223282103,0.14641971643381654,0.03275526742496901,-0.030696657906964218,-0.05050399753520547}; 

/********* Flags globales *********/
char f_cfg = 0; //se pone en 1 luego de la primera vez que cargo el archivo para que la segunda vez se descarte el backlog
char f_pusr2 = 0; //flag de señal SIGUSR2 del padre
char f_cusr2 = 0; //flag de señal SIGUSR2 del hijo
char f_pint = 0; //flag de señal SIGINT del padre
char f_cint = 0; //flag de señal SIGINT del hijo

/********* Variables globales *********/
int cpid = -1; 
int pid_mpu = -1;
int active_conn = 0;  //cantidad de conexiones activas
/********* Prototipos de funciones *********/
//prototipo para que el proceso hijo, pueda procesar la info del cliente. Hacemos un proceso hijo para que, en caso de que haya que atender más de un cliente, podamos aceptarlo
void ProcesarCliente(int s_aux, struct sockaddr_in *pDireccionCliente, int puerto);

//prototipo del proceso hijo que lee los datos del sensor y los carga en la shared memory
int ProcesarDatos(float **, float *, float *,float **, float *, float *,float **, float *, float *, int *);
int ProcesarDatos2(float **xindice, float *xhead, float *xtail,float **yindice, float *yhead, float *ytail,float **zindice, float *zhead, float *ztail, int *elementos);
//prototipo de la función que carga el archivo de config
void load_cfg(int *backlog,int *cant_con, double *coefs,double *coefs2,FILE *fconfig);

//prototipo del handler de la señal SIGUSR2
void sigusr2_phandler();
void sigusr2_chandler();

//prototipo del handler de la señal SIGINT (lo utilizo para que antes de salir, se libere la memoría que solicité)
void sigint_phandler(); //handler de SIGUSR2 para el padre
void sigint_chandler(); //handler de SIGUSR2 para el hijo

void sigchld_handler();

//prototipo de la función encargada del filtrado
void filtrado(float *X, float *Y, float *Z, float *BX,float *BY,float *BZ,  int cant);
void filtrado2(float *X, float *Y, float *Z, float *BX,float *BY,float *BZ,  int cant);

/********* IPC *********/
int shmid; //shared memory id
int semid; //semaphore id
struct sembuf sops[1]; //estructura que utilizo para controlar el semáforo
union semun {			
    int val;
    struct semid_ds *buf;
    ushort *array;
};	//union utilizado en semop
//tengo que crear un semáforo con valor = cantidad de conexiones. Cada proceso que lee, decrementa al valor del semaforo en 1. Cuando el proceso que carga datos lee, pone el valor en 0 => ningún otro puede leer hasta que él lo libere

//tenemos que recibir por consola de comandos el puerto
int main (int argc, char *argv[])
{

    //tenemos que abrir un socket, para escuhar a todos los clientes. Luego, cuando aceptamos un cliente, siempre nos comunciamos con él por medio del segundo socket (el s_aux)
    char ipv4[INET_ADDRSTRLEN];
    int s; //File descriptor del socket que usaremos para escuchar las conexiones
    socklen_t longDirec;
    struct sockaddr_in datosServidor;       //struct sockaddr_in : esta structura hace más fácil referenciar elementos del tipo socket addres 
	int i = 0;
	int count = 0;

	union semun smparg;
	
	//buffers para el proceso hijo que procesa los datos
	float *xindice;
	float *xhead;
	float *xtail;
	float *yindice;
	float *yhead;
	float *ytail;
	float *zindice;
	float *zhead;
	float *ztail;
	int elementos = 0;
	
	//buffer para almacenar los datos que pasaron por el primer filtro
	float *X_F1;
	float *Y_F1;
	float *Z_F1;
	float *X_F1_tail;
	float *Y_F1_tail;
	float *Z_F1_tail;
	float *indice_xf1;
	float *indice_yf1;
	float *indice_zf1;
	
	//varaibles auxiliar para almacenar el dato obtenido del filtro
	float AUX_F1_X;
	float AUX_F1_Y;
	float AUX_F1_Z;
	float AUX_F2_X;
	float AUX_F2_Y;
	float AUX_F2_Z;	
	
	//registro handler de señales
	signal(SIGUSR2,sigusr2_phandler); //registro el handler para la señal SIGUSR2
	signal(SIGINT,sigint_phandler); //registro el handler para la señal SIGINT
	signal(SIGCHLD,sigchld_handler); //registro el handler para la señal SIGCHLD
	
	
	fconfig = fopen("SIGUSR2.txt","r");
	printf("\nHola, archivo cfg!\n");
	if(fconfig == NULL)
	{
		printf("*** Error al abrir el archivo de cfg ***\n Usando valores predeterminados: \n backlog = 100 \n cant_con = 10 \n win_sz = 2 \n");
		backlog = 100;
		cant_con = 10;
		win_sz = 10;
		
	}
	//leo el archivo de cfg
	load_cfg(&backlog,&cant_con,coefs,coefs2,fconfig);
	printf("\nChau, archivo cfg!\n");
	printf("backlog %d\n", backlog);
	printf("cant_con %d\n", cant_con);
	printf("win_sz %d\n", win_sz);
	if(win_sz > 20)
	{
		win_sz = 20;
		printf("win_sz truncado a %d\n", win_sz);
	}
	
	
	/*
		
		int semget(key_t key, int nsems, int semflg);
		
		key: id obtenido con ftok
		nsems: nro de semáfotos que contendrá el set (en nuestro caso 1)
		semflg: flags que indican los permisos a setear en el set de semáforos a crear
		
	*/
	semid = semget(rand(),1,IPC_CREAT | 0660);
	
	/*
		int semctl(int semid, int semnum,int cmd, ... arg  ) --> permite realizar cambios de valor a semáforos individuales o sets completos
		
		semid: id del set obtenido con semget
		semnum: id del semáfoto a manipular
		cmd: acción a realizar sobre el semáfoto (SETVAL: para setear el valor de un semaforo al valor del parámetro "val" de la unión que se le pasa a arg)
		arg: si es necesario, será una unión como la que sigue:
		
				union semun {
					int val;                used for SETVAL only 
					struct semid_ds *buf;  used for IPC_STAT and IPC_SET 
					ushort *array;          used for GETALL and SETALL 
				};	
	*/
	
	smparg.val = 10; //seteo el valor del semáfoto en 10
	
	semctl(semid,0,SETVAL,smparg);
	
	/*
		int semop(int semid, struct sembuf *sops, unsigned int nsops);
		
		semid: id obtenido con semget
		*sops: puntero a la estructura sembuf que llenamos con nuestros comandos
		nsops: indica si estamos enviando más de una estructura sembuf (1 en nuestro caso)
		
		Todas las operaciones para setear, obtener el valor o testear un set de semáforos se hacen con esta función. La funcionalidad de la función, la dicta la estructura sembuf:
	
		struct sembuf {
		ushort sem_num; --> nro de semáforos en el set que queremos manipular
		short sem_op; --> operación a realizar. Varía según sea negativo, positivo o cero: negativo = blockea el proceso llamador hasta que el valor del semáfoto sea mayor o igual al módulo de sem_op. positivo = suma sem_op al valor del semáforo. cero = el proceso llamador esperará hasta que el valor del semáfro sea cero
		short sem_flg; --> 0
		};
		
		
		https://man7.org/linux/man-pages/man2/semop.2.html
		si sempop > 0 => se le agrega este valor al valor de sem_value 
		si sempop < 0 => se resta el valor absoluto de semop de sem_value (si sem_value >= |semop| => la operación puede proseguir)
		
		
	*/
	
    /*
     * 
     *  struct sockaddr_in{
     *      short int           sin_family; -> debe ser AF_INET (IPv4) o AF_INET6 (IPv6)
     *      unsigned short int  sin_port;   -> debe estar en "network byte order" [usamos htons()]
     *      struct in_addr      sin_addr;   -> referencia a struct in_addr, compuesta por un uint32_t que posee la dirección
     *      unsigned char       sin_zero[8]; -> debe setearse a 0 con memset()
     * 
     * };
     * 
     * 
     */
    
  
    //su no recibí la dirección IP y el PUERTO, entocnes me faltan datos.
    if (argc != 2)
    {
        printf("\n\nLinea de comandos: ip:puerto\n\n");
        exit(1);
    }
    
    
    
    // Creamos el socket que usaremos para escuchar. 
    s = socket(AF_INET, SOCK_STREAM,0); //SOCK_STREAM es para TCP. AF_INET es para IPv4
    if (s == -1)
    {
        printf("ERROR: El socket no se ha creado correctamente!\n");
        exit(1);
    }
    
    
    //  Llenamos la estructura sockaddr_in. Asignamos el puerto que queremos y una IP de la maquina.
    datosServidor.sin_family = AF_INET; //IPv4                     
    datosServidor.sin_port = htons(atoi(argv[1])); //atoi = convierte el ASCII de la linea de comandos a entero
    datosServidor.sin_addr.s_addr = htonl(INADDR_ANY);  // INADDR_ANY: le Asignamos cualquiera que sea la IP. Podríamos pasar la de loopback (127.0.0.1), para que sólo se reciban peticiones de esta misma máquina. 0.0.0.0: escucha en todas las direcciones IP de la PC
    
    //int status = 0;
    //if( (status = inet_pton(AF_INET,"10.0.2.15",&(datosServidor.sin_addr)) <= 0 ) )
    //{
    //        printf("ERROR: La dir IP está mal\n");
    //}
    
    
    //NOTAR: también podríamos usar getaddrinfo(), que ayuda a completar las estructuras sockaddr. Para ello podríamos usar una estructura sockaddr_info. Así como está ahora, estamos completando manualmente (que también puede hacerse)
    
    
    //ahora tenemos que hacer bind() para asociar el socket que creamos con el puerto que deseamos escuchar.
    if( bind(s, (struct sockaddr*)&datosServidor, sizeof(datosServidor)) == -1) //bind: para que el puerto (en este caso 55555) quede atrapado para este proceso
    {
        printf("ERROR: este proceso no puede tomar el puerto %s\n",argv[1]);
        exit(1);
    }
    
    inet_ntop(AF_INET,&(datosServidor.sin_addr),ipv4,INET_ADDRSTRLEN);
    printf("\nLa IP del servidor es %s: \n",ipv4);
    printf("\nIngrese en el navegador http://dir ip servidor:%s/\n",argv[1]);
    
    /*una vez realizado el bind(), estamos listos para realizar el listen(). listen() hace que esperemos por conexiones entrantes.
		int listen(sockfd, backlog)
		https://man7.org/linux/man-pages/man2/listen.2.html
		backlog: defines the maximum length to which the
       queue of pending connections for sockfd may grow.  If a
       connection request arrives when the queue is full, the client may
       receive an error with an indication of ECONNREFUSED or, if the
       underlying protocol supports retransmission, the request may be
       ignored so that a later reattempt at connection succeeds.
	
	*/
    ///listen genera el backload: El liente se conecta, pero si bien linux acepta todas las conexiones que entran, puede que el proceso no pueda atenderlos => el SO debe tener una cola de clientes que se conectaron pero que aún no fueron aceptados/atendidos. El largo de esa cola, lo seteamos con "listen" en el campo "backlog"
    
	
    if (listen(s, MAX_CONN) < 0)
    {
        perror("Error en listen");
        close(s);
        exit(1);
    }
    
	
	/*
		Para que los procesos puedan visualzar las mediciones, debemos crear una shared memory. De esta manera se comparte la información entre todos los procesos: 
			proceso de lectura (hace mpu_read()) y procesos de atención de conexiones de clientes HTML
			
			key: una clave
			size: tamaño de memoria compartida
			flags: IPC_CREAT | 0660
	*/
	key_t key = ftok("SIGUSR2.txt", 'a');
	
	shmid = shmget(key, sizeof(float)*6, IPC_CREAT | 0660);
	if(shmid == -1)
	{
		perror("Error en shared mem");
		exit(EXIT_FAILURE);
	}
	printf("Sin error en SHMEM");
	/*
		Debemos crear un nuevo proceso que vaya haciendo el mpu_read()
	*/
	pid_mpu = fork();
	if(pid_mpu < 0)
	{
		printf("\nERROR: no pudo crearse el proceso que lee el MPU\n");
		exit(1);
	}
	if(pid_mpu == 0 ) //si pid_mpu = 0 => estoy en el proceso hijo
	{
		
		int contador = 0;
		//puntero a la dirección de la shared memory
		float *addr;
		//estructura sembuf para operar con el semaforo
		struct sembuf sops;
		
		//adjuntamos la shared memory al proceso
		addr = shmat(shmid, NULL, 0);
		
		signal(SIGUSR2,sigusr2_chandler); //registro el handler para la señal SIGUSR2
		signal(SIGINT,sigint_chandler); //registro el handler para la señal SIGINT
		
		printf("\nID proceso datos = %d\n", getpid());
		//alojo memoria para el buffer con el tamaño de la ventana que debe tener el filtro
		xhead = calloc(win_sz, sizeof(float));
		xtail = xhead + (win_sz -1);
		xindice = xhead;
		
		yhead = calloc(win_sz, sizeof(float));
		ytail = yhead + (win_sz -1);
		yindice = yhead;
		
		zhead = calloc(win_sz, sizeof(float));
		ztail = zhead + (win_sz -1);
		zindice = zhead;
		
		//alojo memoria para el buffer que contiene los datos luego de que pasaron por el primer filtro
		X_F1 = calloc(win_sz, sizeof(float));
		X_F1_tail = X_F1 + (win_sz -1);
		indice_xf1 = X_F1;

		Y_F1 = calloc(win_sz, sizeof(float));
		Y_F1_tail = Y_F1 + (win_sz -1);
		indice_yf1 = Y_F1;
		
		Z_F1 = calloc(win_sz, sizeof(float));
		Z_F1_tail = Z_F1 + (win_sz -1);
		indice_zf1 = Z_F1;
		
		while(1)
		{
			
			if(f_cusr2 == 1)
			{
				free(xhead);
				free(yhead);
				free(zhead);
				printf("backlog %d\n", backlog);
				printf("cant_con %d\n", cant_con);
				printf("win_sz %d\n", win_sz);
				if(win_sz > 20)
				{
					win_sz = 20;
					printf("win_sz truncado a %d\n", win_sz);
				}
				
				xhead = calloc(win_sz, sizeof(float));
				xtail = xhead + (win_sz -1);
				xindice = xhead;

				yhead = calloc(win_sz, sizeof(float));
				ytail = yhead + (win_sz -1);
				yindice = yhead;

				zhead = calloc(win_sz, sizeof(float));
				ztail = zhead + (win_sz -1);
				zindice = zhead;
				
				f_cusr2 = 0;
			}
			
			//me devuelve datos en xhead, yhead , zhead
			if(ProcesarDatos2(&xindice,xhead,xtail,&yindice,yhead,ytail,&zindice,zhead,ztail,&elementos) == 1)
			{
				printf("*** ERROR EN PROCESAR DATOS ***");
			}
			
			printf("\nProcesar datos me devolvió: \n");
			for(contador = 0; contador < win_sz; contador++)
			{
				printf("zhead[%d] = %f\n", contador, *(zhead+contador));
			}
			
			//llamo a filtrado 1
			filtrado( &AUX_F1_X,  &AUX_F1_Y,  &AUX_F1_Z,  xhead, yhead, zhead,   win_sz);
			
			printf("AUX_F1_X = %f", AUX_F1_X);
			printf("\tAUX_F1_Y = %f", AUX_F1_Y);
			printf("\tAUX_F1_Z = %f", AUX_F1_Z);
			
			//guardo en la posición actual del buffer de filtrado 1, los datos que obtuve de la función "filtado()"
			*(indice_xf1) = AUX_F1_X;
			*(indice_yf1) = AUX_F1_Y;
			*(indice_zf1) = AUX_F1_Z;
			
			//incremento los indices del buffer de datos filtrados, para que en la próxima llamada se almacene el dato en la proxima posición del buffer
			indice_xf1+=1;
			indice_yf1+=1;
			indice_zf1+=1;
			
			//si llegué al final del buffer => regreso los indices al inicio
			if(indice_xf1 > X_F1_tail)
			{
				indice_xf1 = X_F1;
				indice_yf1 = Y_F1;
				indice_zf1 = Z_F1;
			}
			
			//llamo a filtrado 2: tengo que pasarle a filtrado 2, los datos que tengo en los buffer donde almaceno los datos del filtrado 1
			filtrado2( &AUX_F2_X,  &AUX_F2_Y,  &AUX_F2_Z,  X_F1, Y_F1, Z_F1,   win_sz);
			
			printf("\nAUX_F2_X = %f", AUX_F2_X);
			printf("\tAUX_F2_Y = %f", AUX_F2_Y);
			printf("\tAUX_F2_Z = %f\n", AUX_F2_Z);
			
			
		/*ahora tengo que volcar los datos en la shared memory. Para eso, utilizo un semáforo como mecanismo de sincronización: 
					le resto 10 (valor del semáforo) cuando voy a escribir en la shared memory, luego le sumo 10 cuando la libero.
					
					Tengo que escribir los datos del filtro 1 y los del filtro 2. Ambos los tengo en las variables auxiliares AUX_F1_X, AUX_F1_Y... AUX_F2_X... AUX_F2_Z
		*/
		sops.sem_num = 0;
		sops.sem_op = -10;
		sops.sem_flg = 0;
		if(semop(semid, &sops, 1) == -1)
		{
			printf("\nError al tomar el semáforo para cargar datos\n");
			exit(0);
		}
		
		printf("\nEl semaforo vale %d \n", semctl(semid, 0, GETVAL));
		*addr     = AUX_F1_X;
		*(addr+1) = AUX_F1_Y;
		*(addr+2) = AUX_F1_Z;
		*(addr+3) = AUX_F2_X;
		*(addr+4) = AUX_F2_Y;
		*(addr+5) = AUX_F2_Z;
		
	
		printf("\naddr[0] = %f\n", 	*(addr)   );
		printf("\n addr[1] = %f\n", *(addr+1) );
		printf("\n addr[2] = %f\n", *(addr+2));
		printf("\n addr[3] = %f\n", *(addr+3));
		printf("\n addr[4] = %f\n", *(addr+4));
		printf("\n addr[5] = %f\n", *(addr+5));
		
		//devuevlo el semáforo porque terminé de escribir la memoria compartida
		sops.sem_op = 10;
		if(semop(semid, &sops,  1) == -1)
		{
			printf("\nError al devolver el semáforo para cargar datos\n");
			exit(0);
		}
		printf("\nEl semaforo vale %d \n", semctl(semid, 0, GETVAL));
			
			
		
		
		if(f_cint)	//si el padre recibió SIGINT => se debe terminar el programa. Para eso salgo del while, libero los recursos que pedí y retorno al punto de llamada, donde se hace exit().
		{
			f_cint = 0;
			
			//antes de salir, cierro el dispositivo
			//close(fd);
			exit(0);
			break;	
		}
		
		//mando al proceso a dormir hasta que haya que refrescar
		sleep(refresh);
			
			
		}	
		exit(0);
	}
	
	
    // Permite atender a multiples usuarios: creamos un proceso hijo que va atendiendo cada conexión mediante el socket auxiliar
    while (1)
    {
        int pid, s_aux;

        struct sockaddr_in datosCliente;
        // La funcion accept rellena la estructura address con
        // informacion del cliente y pone en longDirec la longitud
        // de la estructura.
        longDirec = sizeof(datosCliente);
        s_aux = accept(s, (struct sockaddr*) &datosCliente, &longDirec); //syscall "accept" sirve apra aceptar un cliente. datosCliente, va a tener la IP address y el puerto del client. IMP: &longDirec, hay que inicializarlos con el tamaño del sockaddr_in!!
        if (s_aux < 0)  //s_aux = socket con el que el servidor se comunica con el cliente. Todo lo que es send y Recibe va a s_aux
        {
            perror("Error en accept");
            close(s);
            exit(1);
        }
        active_conn = active_conn + 1; //incremento la cantidad de conexiones que tengo activas
        printf("\n *** cant_con = %d ***", active_conn);
		
		if(active_conn <= 10 )	//cant_con
		{
			pid = fork();   //creo el hijo apra atender el pedido de HTTP
			if (pid < 0)
			{
				perror("No se puede crear un nuevo proceso mediante fork");
				close(s);
				exit(1);
			}
			if (pid == 0)   //si estamos en el caso de proceso hijo, vamos a ProcesarCliente, mandandole el socket que creó "accept" y por últomo le mando el nro de puerto del servidor
			{       // Proceso hijo.
				ProcesarCliente(s_aux, &datosCliente, atoi(argv[1]));
				exit(0);  //una vez que terminé me voy: liquida al hijo, pero no al padre, porque lo estamos ejecutando dentro del hijo
			}
		}
        close(s_aux);  // El proceso padre debe cerrar el socket que usa el hijo.
              
			  
		if(f_pusr2)
		{
			f_pusr2 = 0;
			load_cfg(&backlog,&cant_con,coefs,coefs2,fconfig);

			printf("\n\n \t\t WinSz = %d \n\n", win_sz);

			kill(pid_mpu,12); //envío la señal 12 (SIGUSR2) al proceso hijo que procesa datos, para que recargue sus variables desde el archivo (tamaño de la ventana)
		}
    }

    
    return 0; //no debería llegar nunca porque el server es un while(1)
}



//cómo procesa el hijo el HTTP. Esto es lo específico para HTTP
void ProcesarCliente(int s_aux, struct sockaddr_in *pDireccionCliente, int puerto)
{
    char bufferComunic[8192];
    char ipAddr[20];  //
    int Port;
    int indiceEntrada;
    float tempCelsius;
    int casoRespuesta = 0;
    char HTML[4096];
    char encabezadoHTML[4096];
	struct sembuf sops;
	
	float *addr; //puntero a la dirección de la shared memory
	
  //variables para abrir imagen
    FILE *fp;
    int c, i = 0;
    int sz, szt = 0;
    char *str; 
    int sizeSend = 0;
	//datos x,y,z provenientes del primer filtro
	float dato_shx = 0;
	float dato_shy = 0;
	float dato_shz = 0;
	//datos x,y,z provenientes del segundo filtro
	float dato_shx_2 = 0;
	float dato_shy_2 = 0;
	float dato_shz_2 = 0;
	
  
  //variables para mostrar datos del mpu
	int j = 0;
	int count = 0;
	count = 1;//atoi(argv[1]); 
  
	//adjuntamos la shared memory al proceso
	addr = shmat(shmid, NULL, 0);
	
	//lock semaphore --> https://man7.org/linux/man-pages/man2/semop.2.html
	sops.sem_num = 0;
	sops.sem_op = -1;
	sops.sem_flg = 0;
	if(semop(semid, &sops, 1) == -1)
	{
		printf("\nError al tomar el semáforo para mostrar en datos en el sitio\n");
	}
	
	dato_shx = addr[0];
	dato_shy = addr[1];
	dato_shz = addr[2];
	dato_shx_2 = addr[3];
	dato_shy_2 = addr[4];
	dato_shz_2 = addr[5];
	//unlock semaphore
	sops.sem_num = 0;
	sops.sem_op = 1;
	sops.sem_flg = 0;
	if(semop(semid, &sops, 1) == -1)
	{
		printf("\nError al tomar el semáforo para mostrar en datos en el sitio\n");
	}
  
  strcpy(ipAddr, inet_ntoa(pDireccionCliente->sin_addr)); //ipAddr = ip add del cliente. inet: convierte los 4 bytes de la direccion en el formato legible de X.X.X.X 
  Port = ntohs(pDireccionCliente->sin_port);    //ntohs: network to host short. 
  // Recibe el mensaje del cliente
  //int recv(int sockfd, void *buf, int len, int flags); donde: sockfd = FD del socket a usar. buf = buffer donde guardar info len = largo máxmo del buffer flag = 0
  if (recv(s_aux, bufferComunic, sizeof(bufferComunic), 0) == -1)   //recibo el mensaje del cliente y si hay error, me voy 
  {
    perror("Error en recv");
    exit(1);
  }
  printf("* Recibido del navegador Web %s:%d:\n%s\n\n\n\n",   
         ipAddr, Port, bufferComunic);  //imprimo todo el encabezado del pedido HTTP
  
  
  
   /* Solicitud:
    *   GET / HTTP/1.1
    * 
    * Respuesta:
    * 
    *   HTTP/1.1 200 Ok
        Content-Type: text/html; charset=UTF-8\n
        Content-Length: %d\n El número es: strlen(HTML)
        \n
        "Codigo HTML" 
    * 
    * 
    */
  if (memcmp(bufferComunic, "GET /", 5) == 0)   // "GET /XX" : lo que está luego de "/", está en la quinta posición. memcmp compara los primeros n bytes del string1 con el string2. Deuelve 0 si ambos string son iguales
  {
    
      casoRespuesta = OKMETHOD;
  }
 
  /* Solicitud:
    *   Si el recurso no es /
    * 
    * Respuesta:
    * 
    *  HTTP/1.1 404 Bad resource
    * 
    * 
    */
    if ( ( memcmp(bufferComunic, "GET", 3) == 0 &&   (bufferComunic[4] != '/') )
       // || ( ( (memcmp(bufferComunic, "GET /Archivo-grafico", 20) != 0) && strlen(bufferComunic)>20 ) ) 
    )    // Si no la soliciud no es GET / => Bad resource
    {
        casoRespuesta = BADRESRC;
        //printf("\nel caracter es %c\n", bufferComunic[4]);
        
    }
    
    
    
    
    

     /* Solicitud:
    *   GET /Archivo-grafico HTTP/1.1
    * 
    * Respuesta:
    * 
    *   HTTP/1.1 200 Ok
        Content-Type: image/png\n
        Content-Length: %d\n El número es: sizeof(buffer_imagen)
        \n
        ... acá copiar el gráfico en binario ...
    * 
    * 
    */
    if (memcmp(bufferComunic, "GET /Archivo-grafico", 20) == 0)   // "GET /XX" : lo que está luego de "/", está en la quinta posición. memcmp compara los primeros n bytes del string1 con el string2. Deuelve 0 si ambos string son iguales
    {

        casoRespuesta = GRAPH;
    }   
    
    
    
    
  //armo el HTML dependiendo de en qué caso esté
  
  // Generar HTML.
  // El viewport es obligatorio para que se vea bien en
  // dispositivos móviles.
  
  /* Para dispositivos moviles, siempre poner las siguinetes líneas:
   *
        "<meta name=\"viewport\" "
            "content=\"width=device-width, initial-scale=1.0\">"
   * */
  

  
  switch (casoRespuesta)
  {
    case OKMETHOD:
                    
                    //armo encabezado
//                    sprintf(encabezadoHTML, "<html><head><title>ServerAcc</title>"
//                    "<meta name=\"viewport\" http_equiv = "refresh" content="20" "
//                    "content=\"width=device-width, initial-scale=1.0\">"
//                    "</head>"
//                    "<h1 style=""background-color:purple;""""text-align:center"">Lectura acelerómetro</h1>");
//                    //antes de mandar nada al servidor tengo que armar el HTML, porque para enviar al server tengo que saber el tamño de lo uqe voy a enviar => genero, obtengo el tamño, y luego envío 
//                    sprintf(HTML, "%s<p>Se hizo una solicitd correcta de contenido al servidor</p>",
//                        encabezadoHTML);   //guardo todo en HTML
//                    
//                    
//                    sprintf(bufferComunic,
//                    "HTTP/1.1 200 OK\n"
//                    "Content-Length: %ld\n"
//                    "Content-Type: text/html; charset=utf-8\n"
//                    "Connection: Closed\n\n%s",   //para finalziar el encabezado, sí o sí, poner doble \n . %s es para tener el cuertpo del HTTP, que es todo el HTML ( que está en HTML )
//                    strlen(HTML), HTML);

                    //armo encabezado
                    sprintf(encabezadoHTML, "<html><head><title>ServerAcc</title>"
                    "<meta name=\"viewport\" "
                    "content=\"width=device-width, initial-scale=1.0\">"
					"<meta http-equiv=\"refresh\" content=\"1\">"
                    "</head>"
                    "<h1 style=""background-color:purple;""""text-align:center"">Lectura acelerómetro</h1>");
                    //antes de mandar nada al servidor tengo que armar el HTML, porque para enviar al server tengo que saber el tamño de lo uqe voy a enviar => genero, obtengo el tamño, y luego envío 
                    sprintf(HTML, "%s<p>Se hizo una solicitd correcta de contenido al servidor</p>" 
					"<h2>Los datos luego del primer filtro son:</h2> <p>El valor del dato_shx es: %f g</p>" 
					"<p>El valor del dato_shy es: %f g</p>"
					"<p>El valor del dato_shz es: %f g</p>"
					"<p>El valor de la aceleración total es: %f g</p>"
					"<h2>Los datos luego del segundo filtro son:</h2>" 
					"<p>El valor del dato_shx es: %f g</p> <p>El valor del dato_shy es: %f g</p> <p>El valor del dato_shz es: %f g</p>" 
					"<p>El valor de la aceleración total es: %f g</p>",
                        encabezadoHTML,dato_shx,dato_shy,dato_shz,sqrtf(powf(dato_shx,2)+powf(dato_shy,2)+powf(dato_shz,2)),dato_shx_2,dato_shy_2,dato_shz_2,sqrtf(powf(dato_shx_2,2)+powf(dato_shy_2,2)+powf(dato_shz_2,2)));   //guardo todo en HTML
                    
                    
                    sprintf(bufferComunic,
                    "HTTP/1.1 200 OK\n"
                    "Content-Length: %ld\n"
                    "Content-Type: text/html; charset=utf-8\n"
                    "Connection: Closed\n\n%s",   //para finalziar el encabezado, sí o sí, poner doble \n . %s es para tener el cuertpo del HTTP, que es todo el HTML ( que está en HTML )
                    strlen(HTML), HTML);
                    
					sizeSend = sizeof(bufferComunic);
					
					break;

                    
                    
					//para imprimir variables en HTML probar: document.write(variable)
					
    //para probar: curl -i -X POST http://0.0.0.0:55554/
    case BADMETHOD:
        
                      //armo encabezado
                    sprintf(encabezadoHTML, "<html><head><title>ServerAcc</title>"
                    "<meta name=\"viewport\" "
                    "content=\"width=device-width, initial-scale=1.0\">"
                    "</head>"
                    "<h1>ERROR 400</h1>");
        
                    sprintf(bufferComunic,
                    "HTTP/1.1 400 Bad method\n"
                    "Content-Type: text/html;charset=UTF-8\n"
                    "Content-Length: %ld\n"
                    "Connection: Closed\n\n%s",
                    strlen(encabezadoHTML),encabezadoHTML
                        
                    );
					sizeSend = sizeof(bufferComunic);
                    break;
                    
    case BADRESRC:
                      //armo encabezado
                    sprintf(encabezadoHTML, "<html><head><title>ServerAcc</title>"
                    "<meta name=\"viewport\" "
                    "content=\"width=device-width, initial-scale=1.0\">"
                    "</head>"
                    "<h1>ERROR 404</h1>");
        
                    sprintf(bufferComunic,
                    "HTTP/1.1 404 Bad resourse\n"
                    "Content-Type: text/html;charset=UTF-8\n"
                    "Content-Length: %ld\n"
                    "Connection: Closed\n\n%s",
                    strlen(encabezadoHTML),encabezadoHTML
                        
                    );
					sizeSend = sizeof(bufferComunic);
                    
                    break;
    
    case GRAPH:
        
                    //abro imagen PNG y la almaceno en str 
                    fp = fopen("PNG.png","rb");     //rb: abrimos el archvio en bianrio

                    printf("Se abrió el archivo\n");

                    //obtengo el tamaño del archivo
                    fseek(fp, 0L, SEEK_END);
                    sz = ftell(fp);

                    //vuelvo al inicio del archivo
                    fseek(fp, 0L, SEEK_SET);

                    printf("El tamaño del archivo es: %d\n",sz);

                    //reservo para mi string str un tamaño de memoria que pueda alojar al archivo
                    str = (char *)malloc(sz);

                    //cargo en str el contenido del archivo leído
                    szt = fread(str,sizeof(char), sz,fp);
                    
                    
                    //armo encabezado HTML
                   // sprintf(encabezadoHTML, "<html><head><title>ServerAcc</title>"
                   // "<meta name=\"viewport\" "
                  //  "content=\"width=device-width, initial-scale=1.0\">"
                  //  "</head>"
                 //   );
                    //antes de mandar nada al servidor tengo que armar el HTML, porque para enviar al server tengo que saber el tamño de lo uqe voy a enviar => genero, obtengo el tamño, y luego envío 
                 //   sprintf(HTML, "%s<body>%s</body>",
                 //       encabezadoHTML,str);   //guardo todo en HTML
                    
                    
                    
                    
                    sprintf(bufferComunic,
                    "HTTP/1.1 200 OK\n"
                    "Content-Type: image/png\n"
                    "Content-Length: %d\n"
                    "Connection: Closed\n\n",   //para finalziar el encabezado, sí o sí, poner doble \n . %s es para tener el cuertpo del HTTP, que es todo el HTML ( que está en HTML )
                    sz);
                    
                    //agrego al binario de la imagen, luego del ecabezado HTTP
                    memcpy(bufferComunic+strlen(bufferComunic),str,sz);
                        
                    sizeSend = sizeof(bufferComunic);
                
                    
                    break;
                    
                    
                    
                    
    default:
                    
        
                    sprintf(encabezadoHTML, "<html><head><title>ServerAcc</title>"
                    "<meta name=\"viewport\" "
                    "content=\"width=device-width, initial-scale=1.0\">"
                    "</head>"
                    "<h1>DEFAULT ERROR </h1>");
                    
                    
                   
                    sprintf(bufferComunic,
                    "HTTP/1.1 XXX Default error\n"
                    "Connection: Closed");
                    break;
                    
  }
  
  printf("* Enviado al navegador Web %s:%d:\n%s\n",
         ipAddr, Port, bufferComunic);
  
  // Envia el mensaje al cliente. No usar siempre strlen(bufferComunic), eso es para html. Generar una nueva virable que en el caso de tener que enviar HTML, mande strlen y en el caso de la imagen el sizeof(encabezado)+sizeof(imagen)
  if (send(s_aux, bufferComunic, sizeSend, 0) == -1)   //envío todo lo generado al s_aux
  {
    perror("Error en send");
    exit(1);
  }
  // Cierra la conexion con el cliente actual
  close(s_aux);
}

int ProcesarDatos2(float **xindice, float *xhead, float *xtail,float **yindice, float *yhead, float *ytail,float **zindice, float *zhead, float *ztail, int *elementos)
{
	//buffer de usuario, donde guardaré desde el driver los datos leídos haciendo _copy_to_user 
	uint8_t *buffer;
	
	//buffer para armar el dato a partir de la parte alta y la parte baja entregada por el sensor
	int16_t AZ[2];
	int16_t AY[2];
	int16_t AX[2];
	
	//buffer para pasar el dato a float en unidades de "g"
	float AXF[2]; 
	float AYF[2]; 
	float AZF[2]; 
	

	
	//almacena retorno de mpu_read
	int bytesLeidos;
	
	//file descriptor de mi driver
	int fd;
	
	//cantidad de paquetes a leer
	int count = 2; 
	

	int i = 0,j = 0,h = 0;


	printf("\n estoy en el proceso hijo que lee los datos!\n");
	
	
	//pido memoria para el buffer de usuario en el que almacenaré los datos retornados por el sensor
	buffer = calloc(count,sizeof(uint8_t)*paq_size);
	if(buffer == NULL)
	{	
		printf("\nFallo al allocar memoria para el buffer de usuario\n");
		return 1;
	}
	
	
	//abro el driver del mpu
	fd = open ("/dev/mpu", O_RDWR);
	if(fd < 0)
	{
		perror(" *** Fallo al abrir el dispositivo ***\n");
	}
	
	/*
		Uso la función read definida en el FILE OPERATIONS:
		
			-> Device file operations:
				Las operaciones que se pueden realizar sobre archivos, dependen de los drivers que manejan esos archivos. Esas operaciones están definidas como instancias de "struct file operations".  
					Esta estructura contiene un juego de callbacks, que manejaran cualquier syscall realizada en espacio de usuario sobre un archivo. 
				
				La estructura es: 
					
					struct file_operations {
						struct module *owner;
						lofft (*llseek) (struct file *, loff_t, int);
						ssize_t (*read) (struct file *, char __user *, size_t, loff_t *);
						ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *);
						int (*open) (struct inode *, struct file *);
					};
			
			Cada uno de estos callbacks, está linkeada a una SYSCALL. 
	*/
	
	/*
		*fd: puntero a archivo (abrimos el dispositivo \dev\mpu como un archivo
		buffer: buffer donde escribiremos los datos leídos, para que el usuario pueda tomarlos desde allí
		count: cantidad de paquetes de la transferencia de datos 
	*/
	bytesLeidos = read(fd, buffer, count);
	if(bytesLeidos < 0)
	{	
		printf("\n ** ERROR EN LA LECTURA ** \n");
		return 1;
	}
	else
	{
		
		
	
	
        //buffer[bytesLeidos] = 0;
		printf("\n Cantidad de paquetes copiados: %d\n",bytesLeidos);
       
	   //tomo los valores devueltos por el mpu, que se encuentran divididos en parte alto y baja y arma el dato de cada eje del acelerómetro
		for(i=0;i<count*paq_size;i+=paq_size)
		{
			printf("\nAXH = %x", buffer[i]);
			printf("\nAXL = %x", buffer[i+1]);
			printf("\nAYH = %x", buffer[i+2]);
			printf("\nAYL = %x", buffer[i+3]);
			printf("\nAZH = %x", buffer[i+4]);
			printf("\nAZL = %x\n", buffer[i+5]);
			
			AXH = buffer[i];
			printf("\nAXH = %x\n", AXH);
			AX[j] = (uint16_t) (buffer[i] << 8) + (uint16_t) (buffer[i+1]);
			AY[j] = (uint16_t) (buffer[i+2] << 8) + (uint16_t) (buffer[i+3]);
			AZ[j] = (uint16_t) (buffer[i+4] << 8) + (uint16_t) (buffer[i+5]);
			j++;
			
			
		}
		
		//paso a valores float en unidades de g. El valor 16384 se toma del manual del mpu
		i = 0;
		for(i=0;i<count;i++)
		{
			AXF[i] = (float) AX[i]/16384;
			AYF[i] = (float) AY[i]/16384;
			AZF[i] = (float) AZ[i]/16384;
		}
			
		printf("\nAX = %X", AX[0]);
		printf("\nAXF = %.2f", AXF[0]);
		printf("\nAXF = %.2f", AXF[1]);
		printf("\nAY = %X", AY[0]);
		printf("\nAYF = %.2f", AYF[0]);
		printf("\nAYF = %.2f", AYF[1]);
		printf("\nAZ = %X", AZ[0]);
		printf("\nAZF = %.2f", AZF[0]);
		printf("\nAZF = %.2f\n", AZF[1]);
	
		for(h = 0; h < count; h++)
		{
			**(xindice) = AXF[h];
			**(yindice) = AYF[h];
			**(zindice) = AZF[h];
			
			*(xindice) = *(xindice) + 1;
			*(yindice) = *(yindice) + 1;
			*(zindice) = *(zindice) + 1;
			if(*(xindice) > xtail)
			{
				//vuelvo todos los índices al principio, ya que al moverse todos en simultáneo, si uno llegó al final, todos lo hicieron
				*(xindice) = xhead;
				*(yindice) = yhead;
				*(zindice) = zhead;
				printf("\nEntré a xindice = xhead\n");
			}
			
			if( *(elementos) < win_sz)	 //cantidad de elementos que poseo en el array
			{
				*(elementos) = *(elementos)+1;
			}
		}
		printf("\n BX[0] = %.2f\n", *(xhead));
		printf("\n BX[1] = %.2f\n", *(xhead+1));
		printf("\n BX[2] = %.2f\n", *(xhead+2));
		printf("\n BX[3] = %.2f\n", *(xhead+3));
		printf("\n BY[0] = %.2f\n", *(yhead));
		printf("\n BY[1] = %.2f\n", *(yhead+1));
		printf("\n BZ[0] = %.2f\n", *(zhead));
		printf("\n BZ[1] = %.2f\n", *(zhead+1));
		printf("\n Elementos guardados = %d\n", *(elementos));
		
	}
}


/*
	NO SE USA MÁS! Se cambió por "ProcesarDatos2" 
*/
int ProcesarDatos(float **xindice, float *xhead, float *xtail,float **yindice, float *yhead, float *ytail,float **zindice, float *zhead, float *ztail, int *elementos)
{
	//buffer de usuario, donde guardaré desde el driver los datos leídos haciendo _copy_to_user 
	uint8_t *buffer;

	uint8_t AXL = 0;
	uint8_t AXH = 0;
	uint8_t AYL = 0;
	uint8_t AYH = 0;
	uint8_t AZL = 0;
	uint8_t AZH = 0;

	int16_t *AZ;
	int16_t *AY;
	int16_t *AX;
	float *AXF; 
	float *AYF; 
	float *AZF; 
	
	float *BX;
	float *BY;
	float *BZ;
	
	//vector donde almaceno la sumatoria cuadrática de los datos de aceleración de todos los ejes. Tamaño igual que xhead, yhead & zhead
	float *A = calloc(win_sz, sizeof(float)); 
	
	//vectores para almacenar el segundo filtrado
	float AXF2[10];
	float AYF2[10];
	float AZF2[10];
	
	
	int i = 0;
	int j = 0;
	int h = 0;
	//puntero a la dirección de la shared memory
	float *addr;
	
	//almacena retorno de mpu_read
	int bytesLeidos;
	//file descriptor de mi driver
	int fd;
	int count = 2; //reemplazar esto por win_sz
	struct sembuf sops;
	
	//adjuntamos la shared memory al proceso
	addr = shmat(shmid, NULL, 0);
	printf("\n estoy en el proceso hijo que lee los datos!\n");
	
	
	
	
	
	if(count > 0)
	{
//		buffer = malloc(sizeof(uint8_t)*14*(atoi(argv[1])));
		buffer = calloc(count,sizeof(uint8_t)*paq_size);
		AX = calloc(count,sizeof(int16_t));
		AY = calloc(count,sizeof(int16_t));
		AZ = calloc(count,sizeof(int16_t));
		AXF = calloc(count,sizeof(float));
		AYF = calloc(count,sizeof(float));
		AZF = calloc(count,sizeof(float));
	}
	else
	{
		printf("\nIngrese un nro mayor a 0\n");
		return 1;
	}
	if(buffer == NULL)
	{	
		printf("\nFallo al allocar memoria\n");
		return 1;
	}
	
	//abro el driver del mpu
	fd = open ("/dev/mpu", O_RDWR);
	
	for(i = 0; i<count ;i++)
	{	
		buffer[i]=0;	
		AX[i] = 0;
		AY[i] = 0;
		AZ[i] = 0;
	}


	/*
		Uso la función read definida en el FILE OPERATIONS:
		
			-> Device file operations:
				Las operaciones que se pueden realizar sobre archivos, dependen de los drivers que manejan esos archivos. Esas operaciones están definidas como instancias de "struct file operations".  
					Esta estructura contiene un juego de callbacks, que manejaran cualquier syscall realizada en espacio de usuario sobre un archivo. 
				
				La estructura es: 
					
					struct file_operations {
						struct module *owner;
						lofft (*llseek) (struct file *, loff_t, int);
						ssize_t (*read) (struct file *, char __user *, size_t, loff_t *);
						ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *);
						int (*open) (struct inode *, struct file *);
					};
			
			Cada uno de estos callbacks, está linkeada a una SYSCALL. 
	*/
	if(count > 0 ) 	
	{	
		/*
			*fd: puntero a archivo (abrimos el dispositivo \dev\mpu como un archivo
			buffer: buffer donde escribiremos los datos leídos, para que el usuario pueda tomarlos desde allí
			count: cantidad de paquetes de la transferencia de datos 
		*/
		bytesLeidos = read(fd, buffer, count);
	}
	else
	{
		printf("El nro de paquetes debe ser par mayor a 0");
		return 1;
	}
	
	//read devuelve la cantidad de bytes escritos en el buffer. Si es < 0 => error
	if(bytesLeidos < 0)
	{	
		printf("\n ** ERROR EN LA LECTURA ** \n");
		return 1;
	}
	else
	{
	
        //buffer[bytesLeidos] = 0;
		printf("\n Cantidad de paquetes copiados: %d\n",bytesLeidos);
       
	   

	   //tomo los valores devueltos por el mpu, que se encuentran divididos en parte alto y baja y arma el dato de cada eje del acelerómetro
		for(i=0;i<count*paq_size;i+=paq_size)
		{
			printf("\nAXH = %x", buffer[i]);
			printf("\nAXL = %x", buffer[i+1]);
			printf("\nAYH = %x", buffer[i+2]);
			printf("\nAYL = %x", buffer[i+3]);
			printf("\nAZH = %x", buffer[i+4]);
			printf("\nAZL = %x\n", buffer[i+5]);
			
			AXH = buffer[i];
			printf("\nAXH = %x\n", AXH);
			AX[j] = (uint16_t) (buffer[i] << 8) + (uint16_t) (buffer[i+1]);
			AY[j] = (uint16_t) (buffer[i+2] << 8) + (uint16_t) (buffer[i+3]);
			AZ[j] = (uint16_t) (buffer[i+4] << 8) + (uint16_t) (buffer[i+5]);
			j++;
			
			
		}
		
		//paso a valores float en unidades de g. El valor 16384 se toma del manual del mpu
		i = 0;
		for(i=0;i<count;i++)
		{
			AXF[i] = (float) AX[i]/16384;
			AYF[i] = (float) AY[i]/16384;
			AZF[i] = (float) AZ[i]/16384;
		}
			
		printf("\nAX = %X", AX[0]);
		printf("\nAXF = %.2f", AXF[0]);
		printf("\nAXF = %.2f", AXF[1]);
		printf("\nAY = %X", AY[0]);
		printf("\nAYF = %.2f", AYF[0]);
		printf("\nAYF = %.2f", AYF[1]);
		printf("\nAZ = %X", AZ[0]);
		printf("\nAZF = %.2f", AZF[0]);
		printf("\nAZF = %.2f\n", AZF[1]);
	
		for(h = 0; h < count; h++)
		{
			**(xindice) = AXF[h];
			**(yindice) = AYF[h];
			**(zindice) = AZF[h];
			
			*(xindice) = *(xindice) + 1;
			*(yindice) = *(yindice) + 1;
			*(zindice) = *(zindice) + 1;
			if(*(xindice) > xtail)
			{
				//vuelvo todos los índices al principio, ya que al moverse todos en simultáneo, si uno llegó al final, todos lo hicieron
				*(xindice) = xhead;
				*(yindice) = yhead;
				*(zindice) = zhead;
				printf("\nEntré a xindice = xhead\n");
			}
			
			if( *(elementos) < win_sz)	 //cantidad de elementos que poseo en el array
			{
				*(elementos) = *(elementos)+1;
			}
		}
		printf("\n BX[0] = %.2f\n", *(xhead));
		printf("\n BX[1] = %.2f\n", *(xhead+1));
		printf("\n BX[2] = %.2f\n", *(xhead+2));
		printf("\n BX[3] = %.2f\n", *(xhead+3));
		printf("\n BY[0] = %.2f\n", *(yhead));
		printf("\n BY[1] = %.2f\n", *(yhead+1));
		printf("\n BZ[0] = %.2f\n", *(zhead));
		printf("\n BZ[1] = %.2f\n", *(zhead+1));
		printf("\n Elementos guardados = %d\n", *(elementos));

		
		
		//devuelvo los índices del buffer al comienzo
		BX = xhead;
		BY = yhead;
		BZ = zhead;
		filtrado(AXF,AYF,AZF,BX,BY,BZ,*(elementos));

		
//		addr[0] = 1;
//		addr[1] = 2;

		//tomo el semáforo porque voy a escribir la memoria compartida
		//struct sembuf {
		//ushort sem_num; --> nro de semáforos en el set que queremos manipular
		//short sem_op; --> operación a realizar. Varía según sea negativo, positivo o cero: negativo = blockea el proceso llamador hasta que el valor del semáfoto sea mayor o igual al módulo de sem_op. positivo = suma sem_op al valor del semáforo. cero = el proceso llamador esperará hasta que el valor del semáfro sea cero
		//short sem_flg; --> 0
		
		sops.sem_num = 0;
		sops.sem_op = -10;
		sops.sem_flg = 0;
		if(semop(semid, &sops, 1) == -1)
		{
			printf("\nError al tomar el semáforo para cargar datos\n");
			exit(0);
		}
		printf("\nEl semaforo vale %d \n", semctl(semid, 0, GETVAL));
		*addr   	= *AXF;
		*(addr+1) = *AYF;
		*(addr+2) = *AZF;
		
	
		
		printf("\naddr[0] = %f", 	*(addr)   );
		printf("\n addr[1] = %f", 	*(addr+1) );
		printf("\n addr[2] = %f\n", *(addr+2));
		
		//devuevlo el semáforo porque terminé de escribir la memoria compartida
		sops.sem_op = 10;
		if(semop(semid, &sops,  1) == -1)
		{
			printf("\nError al devolver el semáforo para cargar datos\n");
			exit(0);
		}
		printf("\nEl semaforo vale %d \n", semctl(semid, 0, GETVAL));
		
/**** 	Segundo filtrado	****/
		


	//vuelvo todos los índices al principio
	*(xindice) = xhead;
	*(yindice) = yhead;
	*(zindice) = zhead;
	

	
	
		
		
		free(buffer);
		printf("\nLiberé buffer\n");
		
		free(AX);
		printf("\nLiberé AX\n");
		free(AY);
		printf("\nLiberé AY\n");
		free(AZ);
		printf("\nLiberé AZ\n");
		
		free(AXF);
		printf("\nLiberé AXF\n");
		free(AYF);
		printf("\nLiberé AYF\n");
		free(AZF);
		printf("\nLiberé AZF\n");
	}
		return 0;
}
 
/*
	Función para hacer el primer filtrado
	X, Y, Z: guardan el resultado del filtrado
	BX,BY,BZ: buffer donde están los datos a filtrar
	cant: cantidad de coeficientes. LO uso para realizar luego la división
	
*/
void filtrado(float *X, float *Y, float *Z, float *BX,float *BY,float *BZ,  int cant)
{
		int f = 0;
		float AXF_F = 0;
		float AYF_F = 0;
		float AZF_F = 0;
		//int coefs_aux[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}; //todos los coeficientes del filtro a 1 para empezar. 
		
		for (f=0;f < cant; f++)
		{
			AXF_F += *(BX+f)*(coefs[f]);
			AYF_F += *(BY+f)*(coefs[f]);
			AZF_F += *(BZ+f)*(coefs[f]);
		}
		
		AXF_F = AXF_F/cant;
		AYF_F = AYF_F/cant;
		AZF_F = AZF_F/cant;
		
		//guardo el resultado en la primera posición del array que recibo por referencia. Este resultado luego irá a la memoria compartida
		X[0] = AXF_F;
		Y[0] = AYF_F;
		Z[0] = AZF_F;
}

/*
	Función para hacer el segundo filtrado
*/
void filtrado2(float *X, float *Y, float *Z, float *BX,float *BY,float *BZ,  int cant)
{
		int f = 0;
		float AXF_F = 0;
		float AYF_F = 0;
		float AZF_F = 0;
		int coefs_aux[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}; //todos los coeficientes del filtro a 1 para empezar. 
		
		for (f=0;f < cant; f++)
		{
			AXF_F += *(BX+f)*(coefs_aux[f]);
			AYF_F += *(BY+f)*(coefs_aux[f]);;
			AZF_F += *(BZ+f)*(coefs_aux[f]);;
		}
		
		AXF_F = AXF_F/cant;
		AYF_F = AYF_F/cant;
		AZF_F = AZF_F/cant;
		
		//guardo el resultado en la primera posición del array que recibo por referencia. Este resultado luego irá a la memoria compartida
		X[0] = AXF_F;
		Y[0] = AYF_F;
		Z[0] = AZF_F;
}



void load_cfg(int *backlog,int *cant_con, double *coefs,double *coefs2, FILE *fconfig)
{
		
	char *ptr;
	char line[MAX_LINE_SZ];	
	const char s[2] = ",";
    char *token;
	win_sz = 0;
	
	//regreso el cursor al inicio del archivo
	fseek( fconfig, 0L, SEEK_SET );
	//leo archivo de configuración
	if( fgets (line, MAX_LINE_SZ, fconfig)!=NULL ) 
	{
		if(f_cfg == 0)
		{
			*backlog = strtol(line,&ptr,0); //casteo de char a int
			f_cfg = 1;
		}
	}
	if( fgets (line, MAX_LINE_SZ, fconfig)!=NULL ) 
	{
		*cant_con = strtol(line,&ptr,0); //casteo de char a int
		
	}
	
	//obtengo los coeficientes del primer filtro
	if( fgets (line, MAX_LINE_SZ, fconfig)!=NULL ) 
	{
		//*win_sz = strtol(line,&ptr,0); //casteo de char a int --> lo reemplazo por los coeficientes del filtro según nuevo enunciado
		        printf("line:\n");
	    printf("%s\n\n", line);
//		*win_sz = strtol(line,&ptr,0); //casteo de char a int

        token = strtok(line, s);
        while( token != NULL ) {



            coefs[win_sz] = strtod(token, &ptr);
           // coefs[win_sz] = (double) atof(token);
            printf( " %s\n", token );
            token = strtok(NULL, s);
            win_sz++;
            printf("entre al whie %d veces", win_sz);
            }
			win_sz = win_sz -1;
	}
	win_sz = 0;
		//obtengo los coeficientes del primer filtro
	if( fgets (line, MAX_LINE_SZ, fconfig)!=NULL ) 
	{
		//*win_sz = strtol(line,&ptr,0); //casteo de char a int --> lo reemplazo por los coeficientes del filtro según nuevo enunciado
		printf("line:\n");
	    printf("%s\n\n", line);
//		*win_sz = strtol(line,&ptr,0); //casteo de char a int

        token = strtok(line, s);
        while( token != NULL ) {

		coefs2[win_sz] = strtod(token, &ptr);
	   // coefs[win_sz] = (double) atof(token);
		printf( " %s\n", token );
		token = strtok(NULL, s);
		win_sz++;
		printf("entre al whie %d veces", win_sz);
		}
		//win_sz = win_sz -1;

	}
	
}


void sigusr2_phandler()
{
	//cuando llega siguser2 vengo acá
		printf("\n Recibí SIGUSR2!");
		printf("\n Recargo valores \n");
		load_cfg(&backlog,&cant_con,coefs,coefs2,fconfig);
		f_pusr2 = 1;

}

void sigusr2_chandler() 
{ 
	printf("\nme llegó la señal desde el padre!");
	load_cfg(&backlog,&cant_con,coefs,coefs2,fconfig);
	f_cusr2 = 1;
}



void sigchld_handler()
{
	int pid = -1;
	while ( (pid = waitpid(-1, NULL, WNOHANG)) > 0)		 //https://man7.org/linux/man-pages/man3/wait.3p.html WNOHANG: no suspende ejecución si el resultado no está disponible inmediatamente
	{
		if(pid == pid_mpu)	//waitpid devuelve el pid del proceso hijo que finalizó. Si el proceso que finalizó es el que procesa datos => me voy
		{
			
			printf("\n\t *** PROGRAMA SERVIDOR TERMINADO ***");
			
			_exit(0);
		}
		
		active_conn--; //decremento la cantidad de coneixones que poseo activas
	}
	
}


void sigint_phandler()
{
	//cuando llega siguser2 vengo acá
		printf("\n Recibí SIGINT en el handler del padre!");
		printf("\n Voy a esperar a que el hijo libere los recursos antes de irme \n");
		f_pint = 1;

}

void sigint_chandler() 
{ 
	printf("\nme llegó la señal SIGINT desde el padre!\n");
	f_cint = 1;
}

/*
	Algoritmo de FFT => NO SE UTILIZA (guía vieja)
*/

float* fft(float *x, int N, float m)
{

    float g[N/2]; //vector de muestras pares
	float h[N/2]; //vector de muestras impares

	float * H;
	float * G;
	float * X = malloc(sizeof(float)*N);

    int i = 0;
    char j = 0;
	float pi = 3.14159265359;

    printf("Entré a fft %d m = %2.f\n",N,m);

    if(X == NULL)
    {
        printf("Error al alojar memoria para X en la funcion fft\n");
    }

    if(N == 1)
    {
        //cuando llego a tener un vector con sólo una posición, su DFT es directamente ese valor => lo devuelvo
        printf("llegue al vector con uno solo componente\n");
        printf("X = %f\n\n", *x);
        return x;
    }

    if(N>1)
    {


        //si no estoy en el caso de tener un vector de un sólo valor => sub-divido el vector que tengo en sus partes par e impar y vuelvo a llamar a la función
        i = 0;
        for(i=0;i<N;i+=2)
    	{

    		g[j] = x[i];
    		h[j] = x[i+1];

            j+=1;
    	}
        i = 0;
        for(i=0;i<N/2;i++)
    	{
			//imprimo valores para ver que se hayan cargado correctamente los valores
            printf("g[%d] = %2.f\n", i,g[i]);
            printf("h[%d] = %2.f\n", i,h[i]);
    	}


    //vuelvo a llamar a la función, pasnándole las partes par e impar de mi vector de entrada
    m+=1;
    G =  fft(g,(N/2),m);
    H =  fft(h,(N/2),m);
    printf("llamando fft desde el else\n");

    }
    //una vez que tengo los valores de la etapa siguiente, realizo los cálculos de la etapa actual

    printf("\n\n Regreso a N = %d m =\n\n", N);
    i = 0;
    for(i = 0; i < ((N/2)) ; i++ )
    {
        printf("G[%d] = %f", i,G[i]);
        printf("H[%d] = %f", i,H[i]);
        printf("N = %d", N);
        X[i] = G[i] + (cos((2*pi*i)/(N)))*H[i];
        X[i+(N/2)] = G[i] - (cos((2*pi*i)/(N)))*H[i];


    }
    printf("\n\n Voy a printear N = %d \n\n", N);
    i = 0;
    for(i = 0; i < ((N/2)) ; i++ )
    {
        printf("X[%d] = %f\n\n",i, *(X+i));
        printf("X[%d] = %f\n\n",(i+1), *(X+i+1));
    }

    printf("\n\n Finalizado N = %d \n\n", N);
    return X;	//devuelvo el vector con los resultados de la etapa actual (sub-fft de alguno de los vectores en los que deividí el original)

}




