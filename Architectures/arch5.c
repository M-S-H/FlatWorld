// Architecture 5

/*
	This is an update to architecture 3 where an intensity neuron computes the 
	brightness of each visible object and a winner take all system picks the 
	brightest one to use for training.
*/

int lifetimes[ML];				// Collects simtime
float classification_rms[ML];	// Collects classification rms error
float rms = 0;
int red=0, blue=0, green=0;		// Collects food eaten

float w_oclass[4] = {0.5, 0.5, 0.5, 0.5};	// Initial object classification weights

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
	forwardspeed = 0.05;
	a = w->agents[0] ; /* get agent pointer */
	
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

				// Intensity Neuron / Winner Takes All
				int brightest_value = 0;
				int brightest_index = 0;
				for (i=0; i<31; i++)
				{
					int v_intensity = eyevalues[i][0] + eyevalues[i][1] + eyevalues[i][2];
					if (v_intensity > brightest_value)
					{
						brightest_value = v_intensity;
						brightest_index = i;
					}
				}

				// Eat the object
				delta_energy = eat_colliding_object(w,a,0);

				float desired_value = -1;
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
				float x_classification[4] = {1, eyevalues[brightest_index][0], eyevalues[brightest_index][1], eyevalues[brightest_index][2]};
				for (i=0; i<4; i++)
					v_classification += x_classification[i] * w_oclass[i];

				float y_classification = v_classification;

				float error = desired_value - y_classification;
				rms += error*error;
				for (i=0; i<4; i++)
					w_oclass[i] += 0.1 * error * x_classification[i];
			}

		// Move the Agent
			set_forward_speed_agent( a, forwardspeed ) ;
			move_body_agent( a ) ;


		// decrement metabolic charge by basil metabolism rate.  DO NOT REMOVE THIS CALL
			basal_metabolism_agent(a) ;
		
		simtime++;
		

	} // end agent alive condition
	else
	{		

		//speed[nlifetimes] = forwardspeed;
		//lifetimes[nlifetimes] = simtime;
		printf("Classification Weights: %f, %f, %f, %f\n", w_oclass[0], w_oclass[1], w_oclass[2], w_oclass[3]);

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
		
		
		x = distributions_uniform( Flatworld->xmin, Flatworld->xmax );
		y = distributions_uniform( Flatworld->ymin, Flatworld->ymax );
		h = distributions_uniform( -179.0, 179.0);
		

		/*		
		x = 0;
		y = 0;
		h = a->outstate->body_angle;
		h += 1;
		*/
		

		// Collect Data
		lifetimes[nlifetimes] = simtime;
		rms /= (red+blue+green);
		rms = sqrt(rms);
		classification_rms[nlifetimes] = rms;
		rms = 0;
		

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
			printf("Classification Weights: %f, %f, %f, %f\n", w_oclass[0], w_oclass[1], w_oclass[2], w_oclass[3]);

			printf("Food Eaten:\nRed: %d\tGreen: %d\tBlue: %d\n", red, green, blue);

			// Write out data
			FILE *fp;
			fp = fopen("./Results/Arch5 Lifetimes.csv", "w");
			for(i=0; i<maxnlifetimes; i++)
				fprintf(fp, "%d, %d\n", i, lifetimes[i]);
			fclose(fp);

			fp = fopen("./Results/Arch5 RMS.csv", "w");
			for (i=0; i<maxnlifetimes; i++)
				fprintf(fp, "%d, %f\n", i, classification_rms[i]);
			fclose(fp);

			exit(0) ;
		}
		
		simtime = 0;
		
		
	} /* end agent dead condition */
	
	
}