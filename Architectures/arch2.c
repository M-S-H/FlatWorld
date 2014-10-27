/*
 *  Controller.c
 *  For the UNM Neural Networks class, this should be the only file you will need to modify.
 *  World and agent initialization code are located in the main().  An
 *  example of a non-neural controller is included here.
 *  Note that most all of the functions called here can be found in the 
 *  file FlatworldIICore.c
 *  
 *
 *  Created by Thomas Caudell on 9/15/09.
 *  Modified by Thomas Caudell on 9/30/2010
 *  Modified by Thomas Caudell on 9/13/2012
 *  Modified by Thomas Caudel on 9/10/14
 *  Copyright 2009 UNM. All rights reserved.
 *
 */

 // Architecture 2

float energy[5000];
int red = 0;
int blue = 0;
int green = 0;

void agents_controller( WORLD_TYPE *w )
{ /* Adhoc function to test agents, to be replaced with NN controller. tpc */
	
	AGENT_TYPE *a ;
	int collision_flag=0 ;
	int i,k ;
	int maxvisualreceptor = -1 ;
	int nsomareceptors ;
	int nacousticfrequencies ;
	float delta_energy ;
	float dfb , drl, dth, dh ;
	float headth ;
	float forwardspeed ;
	float maxvisualreceptordirection ;
	float bodyx, bodyy, bodyth ;
	float x, y, h ;
	float **eyevalues, **ear0values, **ear1values, **skinvalues ;
	float ear0mag=0.0, ear1mag=0.0 ;
	time_t now ;
	struct tm *date ;
	char timestamp[30] ;
	
	/* Initialize */
	//forwardspeed = 0.05 * nlifetimes; 
	forwardspeed = 0.1;
	a = w->agents[0] ; /* get agent pointer */
	h = 0.0;
	
	/* test if agent is alive. if so, process sensors and actuators.  if not, report death and 
		 reset agent & world */
	if( a->instate->metabolic_charge>0.0 )
	{	
		// Collision Neuron
		int sensor[3] = {0,1,7};
		int j;

		for (j=0; j<3; j++) 
		{
			collision_flag = read_soma_sensor(w, a);		 	
			skinvalues = extract_soma_receptor_values_pointer( a );
			nsomareceptors = get_number_of_soma_receptors( a );

			float weights[8] = {0, 0, 0, 0, 0, 0, 0, 0};
			weights[sensor[j]] = 1;

			int i;
			float v_col = 0;
			int y_col = 0;

			//printf("\n--------\n");
			for (i=0; i<8; i++)
			{
				v_col += skinvalues[i][0] * weights[i];
				//printf("\tSkinValue: %f", skinvalues[i][0]);
			}
			//printf("\n--------\n");

			if (v_col > 0.0)
				y_col = 1;
			else
				y_col = 0;

			// Eat Neuron
			int v_eat = 0;
			int weight = 1;
			v_eat = y_col * weight;


			if (v_eat > 0)
			{
				k = sensor[j];
				delta_energy = eat_colliding_object(w, a, k);

				if (delta_energy > 0.0)
				{
					//printf ("-- I ate a green!\n");
					green += 1;
				}
				else if (delta_energy < 0.0)
				{
					//printf ("-- I ate a red!\n");
					red += 1;
				}
				else if (delta_energy == 0.0)
				{
					//printf ("-- I ate a blue!\n");
					blue += 1;
				}
			}			
		}

		energy[simtime] = a->instate->metabolic_charge;

		// move the agents body
		set_forward_speed_agent( a, forwardspeed ) ;
		move_body_agent( a ) ;


		// decrement metabolic charge by basil metabolism rate.  DO NOT REMOVE THIS CALL
		basal_metabolism_agent(a) ;
		simtime++ ;

	} // end agent alive condition
	else
	{		

		//speed[nlifetimes] = forwardspeed;
		//lifetimes[nlifetimes] = simtime;

		// Example of agent is dead condition
		printf("agent_controller- Agent has died, eating %d objects. simtime: %d\n",a->instate->itemp[0], simtime ) ;
		now = time(NULL) ;
		date = localtime( &now ) ;
		strftime(timestamp, 30, "%y/%m/%d H: %H M: %M S: %S",date);
		printf("Death time: %s\n",timestamp) ;
		
		// Example as to how to restore the world and agent after it dies. */
		restore_objects_to_world( Flatworld ) ;  /* restore all of the objects back into the world */
		reset_agent_charge( a ) ;               /* recharge the agent's battery to full */
		a->instate->itemp[0] = 0 ;              /* zero the number of object's eaten accumulator */

		x = 0;	//distributions_uniform( Flatworld->xmin, Flatworld->xmax ) ; /* pick random starting position and heading */
		y = 0;	//distributions_uniform( Flatworld->ymin, Flatworld->ymax ) ;
		
		// Slightly Rotate the agent
		h = a->outstate->body_angle;
		h += 20;

		//h = distributions_uniform( -179.0, 179.0) ;

		// Collect Data
		
		printf("Food Eaten:\nRed: %d\tBlue: %d\tGreen: %d\n", red, blue, green);

		printf("\nagent_controller- new coordinates after restoration:  x: %f y: %f h: %f\n",x,y,h) ;
		set_agent_body_position( a, x, y, h ) ;    /* set new position and heading of agent */
		/* Accumulate lifetime statistices */
		avelifetime += (float)simtime ;
		
		nlifetimes++ ;
		if( nlifetimes >= maxnlifetimes )
		{
			avelifetime /= (float)maxnlifetimes ;
			printf("\nAverage lifetime: %f\n",avelifetime);

			printf("Food Eaten:\nRed: %d\tBlue: %d\tGreen: %d\n", red, blue, green);

			// Write out data
			
			/*
			FILE *fp;
			fp = fopen("./Results/Arch2 Energy vs Time.csv", "w");
			int i;
			for(i=0; i<simtime; i++)
			{
				//printf("%d\n", nlifetimes);
				fprintf(fp, "%d, %f\n", i, energy[i]);
			}
			fclose(fp);
			*/

			exit(0) ;
		}
		
		simtime = 0;
		
		
	} /* end agent dead condition */
	
	
}