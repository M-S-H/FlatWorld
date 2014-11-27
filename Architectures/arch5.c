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

 // Architecture 5

// float classification[4] = {-0.024704, -0.370985, 0.497871, -0.222094};
float classification[4] = {0, -0.5622008187649891, 0.7544873346344028, -0.3365673238615887};
float lifetime[360];

void arch5( WORLD_TYPE *w )
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
	forwardspeed = 0.05;
	a = w->agents[0] ; /* get agent pointer */
	h = 0.0;
	
	/* test if agent is alive. if so, process sensors and actuators.  if not, report death and 
		 reset agent & world */

	if( a->instate->metabolic_charge>0.0 )
	{	
		// Movement
		read_visual_sensor(w,a);
		eyevalues = extract_visual_receptor_values_pointer(a, 0);

		float intensities[31];
		int i;

		// Compute Intensities
		for (i=0; i<31; i++) 
		{
			float intensity;
			intensity = 1*eyevalues[i][0] + 1*eyevalues[i][1] + 1*eyevalues[i][2];
			
			float inputs[4] = {1, eyevalues[i][0], eyevalues[i][1], eyevalues[i][2]};
			
			float v = 0;
			int j;
			for (j=0; j<4; j++)
				v += classification[j] * inputs[j];

			int y = 0;
			
			if (v > 0)
				y = 1;

			intensities[i] = (1*y) * (1*intensity);
		}

		// Winner Take All
		int max_intensity_index = 15;
		float max_itensity = 0;
		for (i=0; i<30; i++)
		{
			if (intensities[i] > max_itensity)
			{
				max_itensity = intensities[i];
				max_intensity_index = i;
			}
		}

		// Calculate Angle
		float angles[31] = {-15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
		read_agent_body_position( a, &bodyx, &bodyy, &bodyth );
		set_agent_body_angle(a, bodyth + angles[max_intensity_index]) ;


		// Collision Neuron
		collision_flag = read_soma_sensor(w, a);		 	
		skinvalues = extract_soma_receptor_values_pointer( a );
		nsomareceptors = get_number_of_soma_receptors( a );

		float weights[8] = {1, 0, 0, 0, 0, 0, 0, 0};
		float v_col = 0;
		int y_col = 0;

		for (i=0; i<8; i++)
		{
			v_col += skinvalues[i][0] * weights[i];
		}

		if (v_col > 0.0)
			y_col = 1;
		else
			y_col = 0;

		// Eat Neuron
		int v_eat = 0;
		int weight = 1;
		v_eat = y_col * weight;

		float desired_value = 0;

		if (v_eat > 0)
		{
			read_visual_sensor(w,a);
			eyevalues = extract_visual_receptor_values_pointer(a, 0);

			int i;
			float v = 0;
			float inputs[4] = {1, eyevalues[15][0], eyevalues[15][1], eyevalues[15][2]};
			
			for (i=0; i<4; i++)
				v += classification[i] * inputs[i];

			if (v > 0)
				delta_energy = eat_colliding_object(w, a, 0);
		}		

		// move the agents body
		set_forward_speed_agent( a, forwardspeed ) ;
		move_body_agent( a ) ;

		// decrement metabolic charge by basil metabolism rate.  DO NOT REMOVE THIS CALL
		//for (i=0; i<5; i++)
			basal_metabolism_agent(a) ;
		simtime++ ;

	} // end agent alive condition
	else
	{		
		lifetime[nlifetimes] = simtime;

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
		h += 1;

		printf("\nagent_controller- new coordinates after restoration:  x: %f y: %f h: %f\n",x,y,h) ;
		set_agent_body_position( a, x, y, h ) ;    /* set new position and heading of agent */
		/* Accumulate lifetime statistices */
		avelifetime += (float)simtime ;
		
		nlifetimes++ ;
		if( nlifetimes >= maxnlifetimes )
		{
			avelifetime /= (float)maxnlifetimes ;
			printf("\nAverage lifetime: %f\n",avelifetime);

			// Write out data
			FILE *fp;
			fp = fopen("./Results/Arch4 AvgLifetime.csv", "w");
			int i;
			for(i=0; i<maxnlifetimes; i++)
			{
				fprintf(fp, "%d, %f\n", i, lifetime[i]);
			}
			fclose(fp);

			exit(0) ;
		}
		
		simtime = 0;
		
		
	} /* end agent dead condition */	
}