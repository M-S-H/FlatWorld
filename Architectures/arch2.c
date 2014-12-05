// Architecture 2

/*
	The agent detects touch and will eat anything the mouth comes in
	contact with. There is no learning.
*/

int lifetimes[ML];			// Collects simtime
int red=0, blue=0, green=0;	// Collects food eaten

void arch2( WORLD_TYPE *w )
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
	forwardspeed = 0.05;
	a = w->agents[0]; /* get agent pointer */
	
	if( a->instate->metabolic_charge>0.0 )
	{	
		// Collision Neuron
			collision_flag = read_soma_sensor(w, a);		 	
			skinvalues = extract_soma_receptor_values_pointer( a );
			nsomareceptors = get_number_of_soma_receptors( a );

			float w_collision[8] = {1, 0, 0, 0, 0, 0, 0, 0};

			int i;
			float v_collision = 0;
			int y_collision = 0;

			for (i=0; i<8; i++)
				v_collision += skinvalues[i][0] * w_collision[i];

			if (v_collision > 0.0)
				y_collision = 1;


		// Eat The Object
			if (y_collision > 0)
			{
				delta_energy = eat_colliding_object(w,a,0);
				if (delta_energy > 0.0)
					green += 1;
				else if (delta_energy < 0.0)
					red += 1;
				else if (delta_energy == 0.0)
					blue += 1;
			}

		// Move the agent forward
			set_forward_speed_agent( a, forwardspeed ) ;
			move_body_agent( a ) ;


		// decrement metabolic charge by basil metabolism rate.  DO NOT REMOVE THIS CALL
			basal_metabolism_agent(a) ;
			simtime++ ;

	} // end agent alive condition
	else
	{		
		// Collect Data
		lifetimes[nlifetimes] = simtime;


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

		x = distributions_uniform( Flatworld->xmin, Flatworld->xmax ) ; /* pick random starting position and heading */
		y = distributions_uniform( Flatworld->ymin, Flatworld->ymax ) ;
		h = distributions_uniform( -179.0, 179.0);

		printf("\nagent_controller- new coordinates after restoration:  x: %f y: %f h: %f\n",x,y,h) ;
		set_agent_body_position( a, x, y, h ) ;    /* set new position and heading of agent */
		/* Accumulate lifetime statistices */
		avelifetime += (float)simtime ;
		
		nlifetimes++ ;
		if( nlifetimes >= maxnlifetimes )
		{
			avelifetime /= (float)maxnlifetimes;
			float std = 0;
			int i;
			for (i=0; i<maxnlifetimes; i++)
				std += (lifetimes[i] - avelifetime) * (lifetimes[i] - avelifetime);

			std /= (float)maxnlifetimes;
			std = sqrt(std);
			printf("\nAverage lifetime: %f\tStandard Deviation: %f\n",avelifetime, std);

			printf("Total Food Eaten:\nRed: %d\tBlue: %d\tGreen: %d\n", red, blue, green);
			
			// Liftimes
			FILE *fp;
			fp = fopen("./Results/Arch2 Lifetimes.csv", "w");
			for(i=0; i<maxnlifetimes; i++)
				fprintf(fp, "%d, %d\n", i, lifetimes[i]);
			fclose(fp);

			exit(0);
		}
		
		simtime = 0;
		
		
	} /* end agent dead condition */
}