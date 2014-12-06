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

 // Architecture 13

 /*

	We will use a winner takes all system for each set of intensities

 */

//float classification[4] = {0.266744, -1.744289, 0.950779, 0.948588};
float classification[4] = {-0.024704, -0.370985, 0.497871, -0.222094};
//float classification[4] = {0.235924, -0.388834, 1.016886, -0.389143};
float lifetime[360];
int red = 0;
int blue = 0;
int green = 0;


void arch13( WORLD_TYPE *w )
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

		float intensities[32];
		float green_intensities[31];
		int i;

		float total_intensity = 0;

		// Compute Intensities
		for (i=0; i<31; i++) 
		{
			// Intensity Neuron
			float intensity;
			intensity = 1*eyevalues[i][0] + 1*eyevalues[i][1] + 1*eyevalues[i][2];
			total_intensity += intensity;
			
			float inputs[4] = {1, eyevalues[i][0], eyevalues[i][1], eyevalues[i][2]};
		
			
			// Winner Takes all for contrast enhancement
			int max_component_index = 0;
			float max_component_value = 0;
			int j;
			for (j=1; j<4; j++)
			{
				if (max_component_value < inputs[j])
				{
					max_component_value = inputs[j];
					max_component_index = j;
				}
			}

			inputs[1] = 0;
			inputs[2] = 0;
			inputs[3] = 0;
			inputs[max_component_index] = 1;

			//for (j=0; j<31; j++)
			//printf("%d\t%f,  %f,  %f,  %f\n", i+1, intensity, inputs[0], inputs[1], inputs[2]);
			//printf("%d\t%f,  %f,  %f,  %f\n", i+1, intensity, eyevalues[i][0], eyevalues[i][1], eyevalues[i][2]);
 
			// Classification Neuron
			float v = 0;
			for (j=0; j<4; j++)
				v += classification[j] * inputs[j];

			int y = 0;
			
			if (v > 0)
				y = 1;

			// Gate Neuron
			green_intensities[i] = (1*y) * (1*intensity);
			intensities[i+1] = intensity;
		}

		// Do I see anything? Neuron
		intensities[0] = 0;
		int max_index = 0;
		float max_value = 0;
		int j = 0;

		for (j=0; j<32; j++)
			if (max_value < 0.00001 * intensities[j])
			{
				max_index = j;
				max_value = 0.00001 * intensities[j];
			}

		for (j=32; j<63; j++)
			if (max_value < green_intensities[j-32])
			{
				max_index = j;
				max_value = green_intensities[j-32];
			}


		// Giant winner take all
		float angles[63] = {30, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

		/*
		for (j=0; j<31; j++)
			printf("%d\t%f - %f, %f, %f\n", j+1, green_intensities[j+1], eyevalues[j][0], eyevalues[j][1], eyevalues[j][2]);
		*/

		
		// Winner Take All For Green Intensities
		int max_intensity_index = 0;
		float max_itensity = 0;
		for (i=0; i<32; i++)
		{
			if (green_intensities[i] > max_itensity)
			{
				max_itensity = green_intensities[i];
				max_intensity_index = i;
			}
		}

		// Calculate Angle
		//float angles[32] = {30, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
		

		read_agent_body_position( a, &bodyx, &bodyy, &bodyth );
		set_agent_body_angle(a, bodyth + angles[max_index]) ;


		// Winner Take All For Green Intensities
		max_intensity_index = 15;
		max_itensity = 0;
		for (i=0; i<31; i++)
		{
			if (intensities[i] > max_itensity)
			{
				max_itensity = intensities[i];
				max_intensity_index = i;
			}
		}


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
			{
				delta_energy = eat_colliding_object(w, a, 0);
				if (delta_energy > 0)
					green++;
				else if (delta_energy == 0) {
					blue++;
	
				}
				else 
				{
					red++;
	
				}
			}
		}


		// Energy Level Neuron
		int energy = 0;
		if (a->instate->metabolic_charge > 0.9)
			energy = 1;

		// Max Intensity
		int mi = 0;
		if (green_intensities[15] > 0.9)
			mi = 1;

		// Stop Gate Neuron
		int v_stop = energy + mi;
		int y_stop = 1;
		if (v_stop > 1)
			y_stop = 0;

		forwardspeed = y_stop * forwardspeed;

		//printf("E: %f, MI: %f, FS: %f\n", a->instate->metabolic_charge, green_intensities[15], forwardspeed);


		// move the agents body
		set_forward_speed_agent( a, forwardspeed ) ;
		move_body_agent( a ) ;

		/*
		if (nlifetimes != 3)
			for (i=0; i<25; i++)
				basal_metabolism_agent(a);
		else
			 basal_metabolism_agent(a) ;
		*/

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

		x = distributions_uniform( Flatworld->xmin, Flatworld->xmax ) ; /* pick random starting position and heading */
		y = distributions_uniform( Flatworld->ymin, Flatworld->ymax ) ;
		h = distributions_uniform( -179.0, 179.0) ;

		printf("\nagent_controller- new coordinates after restoration:  x: %f y: %f h: %f\n",x,y,h) ;
		set_agent_body_position( a, x, y, h ) ;    /* set new position and heading of agent */
		/* Accumulate lifetime statistices */
		avelifetime += (float)simtime ;
		
		nlifetimes++ ;
		if( nlifetimes >= maxnlifetimes )
		{
			avelifetime /= (float)maxnlifetimes ;
			printf("\nAverage lifetime: %f\n",avelifetime);

			printf("\nRed: %d\tGreen: %d\tBlue: %d\n", red, green, blue);

			exit(0) ;
		}
		
		simtime = 0;
		
		
	} /* end agent dead condition */	
}