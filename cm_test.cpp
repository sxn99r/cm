/*! 
  \file
  \brief Testing of Coded Modulation (CM) class
  \author Michael Ng

  1.02

  2009/06/24
*/

#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include "itpp/itbase.h"
#include "itpp/itcomm.h"
#include "cm.h"
#include "fileser.h"


/*! 
  \brief This function test the CM module
  
  The following are the details:
  - TTCM rate-2/3, 8PSK, code memory 3, code termination, 4 iterations,
    \ref APPROX_LOG_MAP decoder, symbol-based channel interleaver
  - Flat uncorrelated Rayleigh fading channel
*/

cvec my_channel(int channel_type, int N, double path_factor, TDL_Channel &ray_channel);

int main(int argc, char *argv[])
{
	int i,j;			//Counter variable
	
	bvec b_source_bits, b_decoded_bits;
	ivec i_encoded_symbols;
	cvec c_received_signals;
	ivec No_of_Errors, No_of_Bits, No_of_BlockErrors, No_of_Blocks;
	
	//Channel setup
	//Array<cvec> channel_gains;
	
	cvec channel_gains;
	TDL_Channel ray_channel; // default: uncorrelated Rayleigh fading channel	
	AWGN_Channel awgn_channel; // AWGN channel
	
	cvec source_frame;

	//-------------------------------------------------------CM
	//Coded Modulation module, CM
	CM cm;

	//----read the parameters from the control file
	char *ctrlfile;
	if(argc==1) ctrlfile = "control.cm.par";
	else        ctrlfile = argv[1];
	
	cm.set_parameters_from_file(ctrlfile);
	cm.initialise();
	
	//result file
	FILE *f;  
	char *resultfile= cm.get_result_filename();
	f=fopen(resultfile, "w");
	
	//print out parameters
	cm.print_parameters(stdout);    
	cm.print_parameters(f);
	//-------------------------------------------------------CM	
	
	// read simulation parameters
	FILE *sim_file;
	FILESERVICE fileser;
	sim_file = fopen("control.sim.par", "r");
	if(sim_file==NULL) it_error("control.sim.par not found");
	int max_nrof_frame = fileser.scan_integer(sim_file);
	double ebno_start  = fileser.scan_double(sim_file); 
	double ebno_step   = fileser.scan_double(sim_file);
	double ebno_end    = fileser.scan_double(sim_file);
	int channel_type   = fileser.scan_integer(sim_file);
	double path_sd   = fileser.scan_double(sim_file); 
	double path_sr   = fileser.scan_double(sim_file); 
	double path_rd   = fileser.scan_double(sim_file); 
	fclose(sim_file);
	char text[100];
	sprintf( text, "%f:%f:%f", ebno_start, ebno_step, ebno_end );
	
	//SNR setup
	vec EbN0dB = text;	//Setting the simulation Eb/N0 range in dB
 	//vec EbN0dB = "10:1:15";	//Setting the simulation Eb/N0 range in dB
	
	double A = 1.0;			
	double Ts = 1.0;
	double Ec = A*A*Ts;
	double Eb = cm.compute_Eb(Ec); // calculate energy per databit Eb, from Es based on coding rate and modulation scheme
	
	vec EbN0 = pow(double(10.0), EbN0dB/double(10.0));
	vec N0 = Eb * pow(EbN0, -1.0);
	vec sigma2 = N0/2;

	
	BERC berc;			//BER counter
	BLERC blerc;			//FER counter
	Real_Timer tt;			//Time counter

	vec ber(EbN0dB.length());	//allocate memory for vector to store BER
	vec bler(EbN0dB.length());	//allocate memory for vector to store FER
	
	ber.clear();			//Clear up buffer of BER counter
	bler.clear();			//Clear up buffer of FER counter	
	
	blerc.set_blocksize((long)cm.get_info_bit_length());	//set blocksize of the FER counter
	
	tt.tic();					//Start timer
	
	//RNG_randomize();				//construct random source 
	RNG_reset(0);                                   //reset random seed 
	
	b_source_bits.set_size(cm.get_info_bit_length(), false);
	i_encoded_symbols.set_size(cm.get_sym_length(), false);
	c_received_signals.set_size(cm.get_sym_length(), false);
	b_decoded_bits.set_size(cm.get_info_bit_length(), false);
	
	No_of_Errors.set_size(EbN0dB.length(), false);		//Set the length
	No_of_Bits.set_size(EbN0dB.length(), false);		//for ivectors storing the no	
	No_of_BlockErrors.set_size(EbN0dB.length(),false);	//of errors bits or error frames
	No_of_Blocks.set_size(EbN0dB.length(),false);
			
	No_of_Errors.clear();
	No_of_Bits.clear();
	No_of_BlockErrors.clear();
	No_of_Blocks.clear();

	
	printf("!EbN0(dB)\tSNR(dB)\t\tBER\t\tFER\t\tFrames\n");
	fprintf(f,"!EbN0(dB)\tSNR(dB)\t\tBER\t\tFER\t\tFrames\n");
	
	for(i=0; i< EbN0dB.length(); i++)
	{
		//Set channel noise level
		awgn_channel.set_noise(N0(i));
		
		for(j=0;j<max_nrof_frame;j++)
		{			
		        //Generate random source bits
			b_source_bits = randb(cm.get_info_bit_length());
			
			//CM encode
			cm.encode(b_source_bits, i_encoded_symbols);   
			source_frame = cm.modulate_symbols(i_encoded_symbols);
			
			if(channel_type==-1){ // AWGN channel
			    c_received_signals = awgn_channel(cm.modulate_symbols(i_encoded_symbols));
			    cm.demodulate_soft_symbols(c_received_signals, N0(i));
			}
			else{  // Rayleigh + AWGN channel
			    channel_gains = my_channel(channel_type, cm.get_sym_length(), path_sd, ray_channel);
			    c_received_signals = elem_mult(source_frame, channel_gains);
			    c_received_signals = awgn_channel( c_received_signals );
			    cm.demodulate_soft_symbols(c_received_signals, channel_gains, N0(i)); 
			}
			
			//CM decode
			cm.decode(b_decoded_bits);		
			//cm.decode_using_Pr(b_decoded_bits, cm.get_Pr() );

			berc.clear();
			blerc.clear();
			
			berc.count (b_source_bits, b_decoded_bits);	//Count error bits in a word
			blerc.count (b_source_bits, b_decoded_bits);	//Count frame errors
			
			No_of_Errors(i) += berc.get_errors();		//Updating counters	
			No_of_Bits(i) += berc.get_errors()+berc.get_corrects();
			No_of_BlockErrors(i) +=blerc.get_errors();
			No_of_Blocks(i)++;
			
			if(No_of_Errors(i)>100000)
				break;
		}
		
		ber(i) = (double)No_of_Errors(i)/No_of_Bits(i);
		bler(i) = (double)No_of_BlockErrors(i)/No_of_Blocks(i);

		double Rate = cm.get_coderate() * cm.get_bps();
		double SNRdB = EbN0dB(i) + 10*log10(Rate);


		printf("%f\t%f\t%e\t%e\t%d\n",EbN0dB(i),SNRdB,ber(i),bler(i),j);
		fprintf(f,"%f\t%f\t%e\t%e\t%d\n",EbN0dB(i),SNRdB,ber(i),bler(i),j);
		
		if(ber(i)<1e-6)
		  break;
			
	}

	double time_used = tt.get_time();

	cout<<"!Elapsed time = "<<time_used<<" seconds"<<endl;		//output simulation time
	fprintf(f,"!Elapsed time = %f seconds\n",time_used);		//output simulation time
	tt.toc();							//Stop timer and output simulation time
	
	fclose(f);							//close output file	
	return 0 ;							//exit program
}


cvec my_channel(int channel_type, int N, double path_factor, TDL_Channel &ray_channel){
    Array<cvec> channel_gains;
    int k;
    
    if(channel_type==0){// fast fading
	ray_channel.generate(N, channel_gains);
    }
    else if(channel_type==1){//quasi-static fading
	ray_channel.generate(1, channel_gains);
	channel_gains(0).set_size(N,true);
	for(k=0;k<N;k++) channel_gains(0)[k] = channel_gains(0)[0];
    }
    //cout<<channel_gains<<endl; getchar();
    channel_gains(0) = sqrt(path_factor)*channel_gains(0);

    return channel_gains(0);
}








