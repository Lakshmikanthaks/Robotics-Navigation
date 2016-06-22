//------------------------------------HEADERS--------------------------------///
#include<stdio.h>
#include<stdlib.h>
#include<wiringPi.h>



////////********************Global declarations***************/////////////////


// Use GPIO Pin 17, which is Pin 0 for wiringPi library

#define enc1 27
#define enc2 28


#define	 m11	23
#define	 m12	0
#define  m21	2
#define	 m22	3

// the event counter 
int encCount1 = 0;
int encCount2 = 0;
int i,j;


int q=0;

int value=0;
int i,j;            //Row and column index
char k,z;
int n=9;            //Number of rows and columns (n*n)
int f=0,c=0,v=0;
int nr,nc,pr,pc;    //Next row, next column, previous row, previous column
int a[9][9]={1,1,1,1,1,1,1,1,1,1,81,0,0,0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,0,0,1,0,0,1,1,1,1,0,1,1,1,0,1,1,1,1,0,0,1,1,0,1,1,1,1,1,0,0,2,0,1,1,1,1,1,1,1,1,1,1};      //Array of layout: Starting point=n*n, Target=2, Obstacle=1, free space=0
int m[9][9]={1,1,1,1,1,1,1,1,1,1,2,0,0,0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,0,0,1,0,0,1,1,1,1,0,1,1,1,0,1,1,1,1,0,0,1,1,0,1,1,1,1,1,0,0,49,0,1,1,1,1,1,1,1,1,1,1}; 
int b[9][9];
int sp;
int loopcount=0;

int xy=0;               //Indicates the orientation of robot: xy=0 - Straight, xy=1 - Left, xy=2 - Right, xy=3 - Back
void bs();              //Boundary Scan
void move_straight();
void turn_right();
void turn_left();
void turn_back();
void print_matrix();
void wavefront_algo();
void path_traversal();
char e;

//////////////////////////////////////////kalman variables/////////////////////////////////////////////////////////
void kalman1();
void kalman2();

float pos_est1=0,err_est1=0,k1,pos_obs1=0;
float timein1=0;
float proc_noise1=0.17;  //mean of encoder error correction -q
float err_obs1=0.626;	// variance of encoder corrected trials -r
float err_upd1=0.335;	// -p
float pos_upd1=0;	//position after 0.1 seconds -x
double startTime1=0;



float pos_est2=0,err_est2,k2,pos_obs2=0;
float timein2=0;
float proc_noise2=0.17;  //mean of encoder error correction
float err_obs2=0.626;	// variance of encoder corrected trials
float err_upd2=0.335;	//
float pos_upd2=0;	//position after 0.1 second
double startTime2=0;

float velocity=22;
float error_pos=0;	//error in end position
int error_count;

///////////////////////KALMAN FUNCTION/////////////
void kalman1()
{
	timein1 = (micros() - startTime1)/1000000;
	
	
	
	//printf("time = %f\n",timein1);
	startTime1 = micros();

	pos_est1=pos_upd1+timein1*velocity;
	//printf("position estimation = %f\n",pos_est1);
	err_est1=err_upd1+proc_noise1;
	
	//printf("Error in estimation = %f\n",err_est1);
	
	k1=err_est1/(err_est1+err_obs1);
	
	
	//printf("\t kalman gain = %f\n",k1);

	//printf("Obseved value = %f\n",pos_obs1);
	pos_upd1=pos_est1+k1*(pos_obs1-pos_est1);
	//printf("position update = %f\n",pos_upd1);
	
	err_upd1=(1-k1)*err_est1;
	//printf("error update= %f\n\n\n",err_upd1);

}

void kalman2()
{
	timein2 = (micros() - startTime2)/1000000;
	

	//printf("\t\t\ttime = %f\n",timein2);
	startTime2 = micros();

	pos_est2=pos_upd2+timein2*velocity;
	//printf("\t\t\tposition estimation = %f\n",pos_est2);
	err_est2=err_upd2+proc_noise2;
	
	//printf("\t\t\tError in estimation = %f\n",err_est2);
	
	k2=err_est2/(err_est2+err_obs2);
	
	
	//printf("\t\t\t\t kalman gain = %f\n",k2);

	//printf("\t\t\tObseved value = %f\n",pos_obs2);
	pos_upd2=pos_est2+k2*(pos_obs2-pos_est2);
	//printf("\t\t\tposition update = %f\n",pos_upd2);
	
	err_upd2=(1-k2)*err_est2;
	//printf("\t\t\terror update= %f\n\n\n",err_upd2);
}





//////////////-----------------BOUNDARY SCAN----------------/////////////////

void bs()               //Boundary Scan function
{
    value=sp;
    if((a[i-1][j]>1)&& (a[i-1][j]!=sp))
    {
        value=a[i-1][j] +1;
    }
    if((a[i+1][j]>1)&&(a[i+1][j]!=sp))
    {

        if(a[i+1][j] < value )
        {
           value=a[i+1][j]+1 ;
        }
    }
    if((a[i][j-1]>1)&&(a[i][j-1]!=sp))
    {


        if(a[i][j-1]<value)
           {
                 value=a[i][j-1] +1;
           }
    }
    if((a[i][j+1]>1)&&(a[i][j+1]!=sp))
    {


        if(a[i][j+1]<value)
        {
            value=a[i][j+1] +1;
        }
    }


    if(value==sp)
    {
        value=0;
    }
    else
        {
    a[i][j]=value;
    }

}

/////////////*****************Print Matrix*********///////////

void print_matrix()
{
    for(i=0;i<n;i++)
        {
            for(j=0;j<n;j++)
            {
                printf("%d",a[i][j]);
                printf("\t");
            }
            printf("\n");
        }
}

////////////////////////////-----------ALGO--------------------///////////////

void wavefront_algo()
{
	//printf("\nwavefront");

while(f==0)
{

    for(j=0;(j<n)&&(f==0);j++) 
    {
        for(i=0;(i<n)&&(f==0);i++)
        {
            if((a[i][j]!=1) && (a[i][j]!=2))
            {
                if(a[i][j]==sp)
                {
                    bs();
                    if(value>2)
                        {
                            //printf("Path found\n");
                            f=1;
                            nr=i;
                            nc=j;
                        }
		else if(loopcount>n*n)
		{
			//printf("No path exists\n");
			//printf("Enter a key to exit\n");
			//scanf("%c",&e);
			exit(0);
		}

		else
                {
                        a[i][j]=sp;
                }
	    }

                else
                {
                    bs();
                }

            }

        }
    }
	
	loopcount++;
}
}

//////////////----------------Path traversal--------/////////////////////
void path_traversal()
{

v=a[nr][nc];
i=nr,j=nc;

//printf("\npress a key\n");
//scanf("%c",&z);
//printf("\n");
//printf("The path traversal sequence :\n");
//printf("%d\t",a[nr][nc]);
//printf("( %d , %d )\n", nr,nc);

while(v>2)
{
pr=nr;
pc=nc;
    if(a[i-1][j]==(v-1))
    {
        nr=i-1;
        nc=j;
    }
    else if(a[i+1][j]==(v-1))
    {
        nr=i+1;
        nc=j;
    }
    else if(a[i][j-1]==(v-1))
    {
        nr=i;
        nc=j-1;
    }
    else if(a[i][j+1]==(v-1))
    {
        nr=i;
        nc=j+1;
    }
    else
        printf("Error\n");



    if(nr==pr-1)
    {
        // cout<<a[nr][nc]<<"      ";
        // cout<<"( "<<nr<<", "<<nc<<" )"<<endl;

        if(xy==0)
        {
            move_straight();
            xy=xy;
        }
        else if(xy==1)
        {
            turn_right();
            move_straight();
            xy=0;
        }
        else if(xy==2)
        {
            turn_left();
            move_straight();
             xy=0;
        }
        else if(xy==3)
        {
            turn_back();
            move_straight();
             xy=0;
        }
    }

    else if(nr==pr+1)
    {
        // cout<<a[nr][nc]<<"      ";
        // cout<<"( "<<nr<<", "<<nc<<" )"<<endl;

        if(xy==0)
        {
            turn_back();
            move_straight();
            xy=3;
        }
        else if(xy==1)
        {
            turn_left();
            move_straight();
            xy=3;
        }
        else if(xy==2)
        {
            turn_right();
            move_straight();
             xy=3;
        }
        else if(xy==3)
        {
            move_straight();
            xy=3;
        }

    }
    else if(nc==pc-1)
    {
       // cout<<a[nr][nc]<<"      ";
       // cout<<"( "<<nr<<", "<<nc<<" )"<<endl;

        if(xy==0)
        {
            turn_left();
            move_straight();
            xy=1;
        }
        else if(xy==1)
        {
            move_straight();
            xy=1;
        }
        else if(xy==2)
        {
           turn_back();
           move_straight();
             xy=1;
        }
        else if(xy==3)
        {
            turn_right();
            move_straight();
            xy=1;
        }

    }
    else if(nc==pc+1)
    {
       // cout<<a[nr][nc]<<"      ";
       // cout<<"( "<<nr<<", "<<nc<<" )"<<endl;

        if(xy==0)
        {
            turn_right();
            move_straight();
            xy=2;
        }
        else if(xy==1)
        {
            turn_back();
            move_straight();
            xy=2;
        }
        else if(xy==2)
        {
            move_straight();
             xy=2;
        }
        else if(xy==3)
        {
            turn_left();
            move_straight();
            xy=2;
        }

    }


    i=nr;
    j=nc;
    v=v-1;
}
printf("\n");
printf("target reached ^*^*^*\n");
}

////////////---------------Move-----------------/////////////


void move_straight()
    {
   // printf("\nMove Straight");
	delay(1000);
	encCount1 = 0;
    encCount2 = 0;
    
pos_est1=0;
err_est1=0;
pos_obs1=0;
pos_upd1=0;	//position after 0.1 seconds -x

pos_est2=0;
err_est2=0;
pos_obs2=0;
pos_upd2=0;	//position after 0.1 second
	
	digitalWrite(m11,0);
	digitalWrite(m12,1);
	digitalWrite(m21,1);
	digitalWrite(m22,0);
	
	while(encCount1<=960);
	
	digitalWrite(m11,0);
	digitalWrite(m12,0);
	digitalWrite(m21,0);
	digitalWrite(m22,0);
	
	/*startTime1 = micros();
	startTime2 = micros();


while(pos_upd2<=60)
{
//printf("\n%f",pos_upd1);
//printf("\t%f",pos_upd2);	
	kalman1();
	kalman2();

	
if(q>40 && q<60)
	{
		//printf("\nhi\n");
	while(pos_upd1<pos_upd2)
	{
	//printf("\nCorrecting\n");	
	digitalWrite(m11,0);
	digitalWrite(m12,1);
	digitalWrite(m21,0);
	digitalWrite(m22,0);
	kalman1();

    }
    digitalWrite(m21,1);
	digitalWrite(m22,0);
  q=0;
}

}

	digitalWrite(m11,0);
	digitalWrite(m12,0);
	digitalWrite(m21,0);
	digitalWrite(m22,0);
	
q=0;

//error_pos = error_pos + (pos_upd2-60);	

pos_est1=0;
err_est1=0;
pos_obs1=0;
pos_upd1=0;	//position after 0.1 seconds -x

pos_est2=0;
err_est2=0;
pos_obs2=0;
pos_upd2=0;	//position after 0.1 second
*/
return;
}

void turn_left()
    {
        //printf("\nTurn left");
	delay(1000);
	encCount1 = 0;
    encCount2 = 0;
	

	
	digitalWrite(m11,1);
	digitalWrite(m12,0);
	digitalWrite(m21,1);
	digitalWrite(m22,0);
	
	while(encCount2<=470);
	

	digitalWrite(m11,0);
	digitalWrite(m12,0);
	digitalWrite(m21,0);
	digitalWrite(m22,0);

    }

void turn_right()
    {
      //  printf("\nTurn right");

	delay(1000);
	encCount1 = 0;
    encCount2 = 0;
	
	
	digitalWrite(m11,0);
	digitalWrite(m12,1);
	digitalWrite(m21,0);
	digitalWrite(m22,1);
	
	while(encCount1<=465);

	digitalWrite(m11,0);
	digitalWrite(m12,0);
	digitalWrite(m21,0);
	digitalWrite(m22,0);

    }

void turn_back()
    {
       // printf("\nMove Back");
delay(1000);
	encCount1 = 0;
    encCount2 = 0;
   
    
	digitalWrite(m11,1);
	digitalWrite(m12,0);
	digitalWrite(m21,1);
	digitalWrite(m22,0);
	
	while(encCount1<=930);

	digitalWrite(m11,0);
	digitalWrite(m12,0);
	digitalWrite(m21,0);
	digitalWrite(m22,0);

    }

////////////////////---ISR---////////////////////

void encInturrupt1(void) {
   encCount1++;
   pos_obs1 = (100*encCount1)/1600.0	;		//1650 encoder count for 100cm
   q++;
   // printf( "%d\n", encCount1);
}

void encInturrupt2(void) {
   encCount2++;
   pos_obs2 = (100*encCount2)/1600.0	;		//1650 encoder count for 100cm
   // printf("\t%d\n", encCount2);
}


/////////////******************Main**************////////////////


int main()
{

	
	  wiringPiSetup(); 
 

  // set Pin 17/0 generate an interrupt on high-to-low transitions
  // and attach myInterrupt() to the interrupt
 
	wiringPiISR(enc1, INT_EDGE_FALLING, &encInturrupt1) ;
	wiringPiISR(enc2, INT_EDGE_FALLING, &encInturrupt2) ;
  
  // display counter value every second.
  
 
    pinMode(23,OUTPUT);
	pinMode(0,OUTPUT);
	pinMode(2,OUTPUT);
	pinMode(3,OUTPUT);

delay(5000);
sp=n*n;
   // printf("Initial grid matrix\n");
    //print_matrix();

for(i=0;i<n;i++)        //backup of initial matrix
        {
            for(j=0;j<n;j++)
            {
                b[i][j]=a[i][j];
            }
        }

wavefront_algo();           //Run wavefront algorithm



//printf("\nEnter any character key\n");
//scanf("%c",&k);

//printf("Final grid Matrix\n");
//print_matrix();                                                      //Output final matrix

path_traversal();

//printf("\nPress a key and then enter to come back");
//scanf("%c",&e);

delay(5000);

////////////////////***** come back ************///////


for(i=0;i<n;i++)        //backup of initial matrix
        {
            for(j=0;j<n;j++)
            {
                a[i][j]=m[i][j];
            }
        }

i=0;
j=0;

f=0;
c=0;
v=0;
value=0;

loopcount=0;

 //printf("\ncome back Initial grid matrix\n");
   // print_matrix();

/*for(i=0;i<n;i++)        //backup of initial matrix
        {
            for(j=0;j<n;j++)
            {
                b[i][j]=a[i][j];
            }
        }*/

wavefront_algo();           //Run wavefront algorithm



//printf("\nEnter any character key\n");
//scanf("%c",&k);

//printf("Final grid Matrix\n");
//print_matrix();                                                      //Output final matrix

path_traversal();

//printf("\nPress a key and then enter to exit");
//scanf("%c",&e);

return 0;
}

