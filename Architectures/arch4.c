// Architecture 4

/*
	The agent continues to move in a single direction at a constant speed
	but will now classify the objects it contacts and will only eat the 
	objects it associates with a reward
*/

int lifetimes[ML];			// Collects simtime
int red=0, blue=0, green=0;	// Collects food eaten

float w_oclass[4] = {0.235924, -0.388834, 1.016886, -0.389143};

void arch4( WORLD_TYPE *w )
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
	
	/* test if agent is alive. if so, process sensors and actuators.  if not, report death and 
		 reset agent & world */
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



		// Classify and Eat the object
			if (y_collision > 0)
			{
				// Read eye values before eating
				read_visual_sensor(w,a);
				eyevalues = extract_visual_receptor_values_pointer(a,0);

				// Classification Neuron
					int j;
					float v_classification = 0, y_classification = 0;
					float x_classification[4] = {1, eyevalues[15][0], eyevalues[15][1], eyevalues[15][2]};
					for (j=0; j<4; j++)
						v_classification += w_oclass[j] * x_classification[j];

					// Eat if classified as a reward
					if (v_classification > 0)
						y_classification = 1;

				// Eat Neuron
					float delta_energy = 0;
					if (y_classification > 0)
						delta_energy = eat_colliding_object(w,a,0);

					if (delta_energy > 0)
						green++;
					else if (delta_energy < 0)
						red++;
					else
						blue++;

				/*
				// Eat the object
				delta_energy = eat_colliding_object(w,a,0);

				float desired_value = 0;
				if (delta_energy > 0.0)
				{
					desired_value = 1.0;
					green++;
				}
				else if (delta_energy < 0.0)
					red++;
				else
					blue++;

				// Training the classification neuron
				float v_classification = 0;
				float x_classification[4] = {1, eyevalues[15][0], eyevalues[15][1], eyevalues[15][2]};
				for (i=0; i<4; i++)
					v_classification += x_classification[i] * w_oclass[i];

				float y_classification = v_classification;

				float error = desired_value - y_classification;
				rms += error*error;
				for (i=0; i<4; i++)
					w_oclass[i] += 0.1 * error * x_classification[i];
				*/
			}

		/*
		// Collision Neuron

		collision_flag = read_soma_sensor(w, a);		 	
		skinvalues = extract_soma_receptor_values_pointer( a );
		nsomareceptors = get_number_of_soma_receptors( a );

		float weights[8] = {1, 0, 0, 0, 0, 0, 0, 0};

		int i;
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
				v += w_oclass[i] * inputs[i];

			if (v > 0)
				delta_energy = eat_colliding_object(w, a, 0);
		}
		*/		

		// move the agents body
			set_forward_speed_agent(a, forwardspeed) ;
			move_body_agent(a) ;

		// decrement metabolic charge by basil metabolism rate.  DO NOT REMOVE THIS CALL
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