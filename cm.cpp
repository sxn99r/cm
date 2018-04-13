/*! 
  \file 
  \brief Implementation of Coded Modulation (CM) class
  \author Michael Ng

  1.03

  2009/06/24
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include "itpp/itcomm.h"

#include "itpp_ctrl.h"
#include "cm.h"
#include "fileser.h"

CM::~CM()
{
    int i;

    if(mode==TCM){ 
      delete tcm; delete [] map->Ip1; delete map; 
      delete modulation;
      delete interleaver;
    }
    else if(mode==TTCM){ 
      delete tcm; delete [] map->Ip1; delete [] map->Ip2; delete map; delete ttcm->interleaver; delete ttcm; 
      delete modulation;
      delete interleaver;
    }
    else if(mode==BICM || mode==BICMID){ 
      int max_intlv=8;
      if(bicm->n>8) max_intlv=bicm->n;
      for(i=0;i<max_intlv;i++) delete bicm->bit_interleaver[i];
      delete[] bicm->bit_interleaver;
      
      delete[] siso->Apo_data_bit;
      delete[] siso->Apr_coded_bit;
      delete[] siso->Apo_coded_bit;
      
      delete siso; delete bicm; delete bicmid; 
      delete modulation;
      delete interleaver;    
    }
}


/*-----------CM parameter setup----------------------------------*/
void CM::set_parameters_from_file(char *fname)
{
  FILE *f;
  FILESERVICE fileser;

  f = fopen(fname, "r");
  int mode_in = fileser.scan_integer(f);
  int k_in    = fileser.scan_integer(f);
  int n_in    = fileser.scan_integer(f);
  int L_in    = fileser.scan_integer(f);
  int Terminated_in = fileser.scan_integer(f); 
  int iterations_in = fileser.scan_integer(f); 
  int decoder_type_in     = fileser.scan_integer(f);
  int no_of_symbols_in    = fileser.scan_integer(f);
  int modulation_type_in  = fileser.scan_integer(f);
  int interleaver_mode_in = fileser.scan_integer(f);

  fileser.scan_text(f, result_filename);
  
  set_parameters(mode_in, k_in, n_in, L_in, Terminated_in, 
		 iterations_in, decoder_type_in, no_of_symbols_in, 
		 modulation_type_in, interleaver_mode_in);
  
  fclose(f);
}

void CM::set_parameters(const int mode_in, const int k_in, const int n_in, const int L_in, const int Terminated_in, 
			const int iterations_in, const int decoder_type_in, const int no_of_symbols_in, 
			const int modulation_type_in, const int interleaver_mode_in)
{
  mode=mode_in;
  if(mode!=TCM && mode!=TTCM && mode!=BICM & mode!=BICMID) s_error("coded modulation mode: TCM, TTCM, BICM or BICMID");
  
  k=k_in;
  n=n_in;
  if(n!=k+1) s_error("make sure n=k+1: such that the modulation level is doubled but experience no bandwidth expansion");

  L=L_in;
  
  Terminated=Terminated_in;
  if(Terminated!=ON && Terminated!=OFF) s_error("coder termination: ON or OFF");
  
  iterations=iterations_in;
  if(iterations==0) if(mode==TTCM || mode==BICMID) s_error("turbo iterations for TTCM or BICMID must > 0");

  decoder_type=decoder_type_in;
  if(decoder_type!=CM_EXACT_LOG_MAP && decoder_type!=CM_APPROX_LOG_MAP && decoder_type!=CM_MAX_LOG_MAP )
    s_error("decoder type: CM_EXACT_LOG_MAP, CM_APPROX_LOG_MAP or CM_MAX_LOG_MAP");
  
  max_no_of_symbols = no_of_symbols_in;
  no_of_symbols     = no_of_symbols_in;
  
  modulation_type = modulation_type_in;
  if(modulation_type!=CPSK && modulation_type!=CQAM) s_error("modulation type: CPSK or CQAM");
  
  interleaver_mode = interleaver_mode_in;
  if(interleaver_mode!=OFF && interleaver_mode!=ON && interleaver_mode!=IQ_INTLV ) s_error("interleaver mode: OFF, ON or IQ_INTLV");
  
#ifdef debug_cm
 printf("%d %d %d %d %d %d %d %d %d ",
	 mode,k,n,L,Terminated,iterations,decoder_type,no_of_symbols,modulation_type); getchar();
#endif
  
  if(Terminated==1){ 
    if(mode==TTCM) no_of_info_symbols=no_of_symbols_in-2*L;
    else           no_of_info_symbols=no_of_symbols_in-L;
  }
  else no_of_info_symbols=no_of_symbols_in;
  
  no_of_input_bits = no_of_symbols * k;
  no_of_info_bits = no_of_info_symbols * k;
  no_of_coded_bits = no_of_symbols * n;
  no_of_tail_bits = (no_of_symbols - no_of_info_symbols) * k;
  
}

void CM::print_parameters(FILE *fp)
{
  fprintf(fp, "!Coded Modulation:\n");
  if(mode==TCM)        fprintf(fp, "!  TCM    (k=%d, n=%d), code memory=%d\n", k,n,L);
  else if(mode==TTCM)  fprintf(fp, "!  TTCM   (k=%d, n=%d), code memory=%d, iteration number=%d\n", k,n,L,iterations);
  else if(mode==BICM)  fprintf(fp, "!  BICM   (k=%d, n=%d), code memory=%d\n", k,n,L);
  else if(mode==BICMID)fprintf(fp, "!  BICMID (k=%d, n=%d), code memory=%d, iteration number=%d\n", k,n,L,iterations);
  
  if(decoder_type==CM_EXACT_LOG_MAP)       fprintf(fp, "!  Decoder type: exact Log MAP\n");
  else if(decoder_type==CM_APPROX_LOG_MAP) fprintf(fp, "!  Decoder type: approximated Log MAP\n");
  else if(decoder_type==CM_MAX_LOG_MAP)    fprintf(fp, "!  Decoder type: maximum Log MAP\n");
  
  if(mode==TCM || mode==TTCM){
    int i;
    fprintf(fp, "!  Gen_poly = ");
    for(i=0;i<n;i++) fprintf(fp, "%d ", tcm->GenPoly[i]);
    fprintf(fp,"\n");
  }
  else{
    int i,j;
    if(bicm->puncture_code == OFF){
      for(i=0;i<k;i++){
	fprintf(fp, "!  Gen_poly[%d] = ",i);
	for(j=0;j<n;j++) fprintf(fp,"%d ", bicm->GenPoly(i,j));
	fprintf(fp,"\n");
      }
    }
    else{
      fprintf(fp, "!  Puncturing Pattern = ");
      for(j=0; j<k; j++) fprintf(fp, "%d ", bicm->puncture[j]&1);
      fprintf(fp, "\n!                       ");
      for(j=0; j<k; j++) fprintf(fp, "%d ", (bicm->puncture[j]>>1) & 1);
      fprintf(fp, "\n");
      
      fprintf(fp, "!  Gen_poly[0] = ");
      for(j=0;j<2;j++) fprintf(fp, "%d ", bicm->GenPoly(0,j));
      fprintf(fp, "\n");
    }
  }
  
  fprintf(fp, "!  Code termination: %s\n", Terminated==ON ? "ON":"OFF");
  fprintf(fp, "!  Block length: output coded (modulated) symbol (%d), input data symbol (%d)\n", no_of_symbols, no_of_info_symbols);
  
  fprintf(fp, "!  Modulation: %d-%s %s\n", modulation->number_of_levels, 
	  modulation_type==CPSK ? "PSK": "QAM", modulation->mode==SP ? "Set Partition":"GRAY");
  
  if(mode==TTCM)                 
    fprintf(fp, "!  Turbo interleaver  : 1 odd-even separated random symbol interleaver of length %d: Intlv.oes.%d\n", no_of_symbols,no_of_symbols);    
  if(mode==BICM || mode==BICMID) 
    fprintf(fp, "!  Channel interleaver: %d parallel random bit interleavers of length %d: Bit[0-%d]Intlv.%d\n", n,no_of_symbols,n,no_of_symbols);  
  if(mode==TCM || mode==TTCM){
    if(interleaver_mode==OFF)    fprintf(fp, "!  Channel interleaver: OFF\n");
    if(interleaver_mode==ON)     fprintf(fp, "!  Channel interleaver: 1 random symbol interleaver of length %d: Intlv.%d\n", no_of_symbols,no_of_symbols);
  }
  if(interleaver_mode==IQ_INTLV) fprintf(fp, "!  Channel interleaver: 2 random IQ interleavers of length %d: Intlv[I,Q].%d\n", no_of_symbols,no_of_symbols);
  
  fprintf(fp, "!\n");
}

void CM::print_parameters(fstream &fs)
{
  fs<<"!Coded Modulation:"<<endl;
  if(mode==TCM)        fs<<"!  TCM    (k="<<k<<", n="<<n<<"), code memory="<<L<<endl;
  else if(mode==TTCM)  fs<<"!  TTCM   (k="<<k<<", n="<<n<<"), code memory="<<L<<", iteration number="<<iterations<<endl;
  else if(mode==BICM)  fs<<"!  BICM   (k="<<k<<", n="<<n<<"), code memory="<<L<<endl;
  else if(mode==BICMID)fs<<"!  BICMID (k="<<k<<", n="<<n<<"), code memory="<<L<<", iteration number="<<iterations<<endl;
  
  if(decoder_type==CM_EXACT_LOG_MAP)       fs<<"!  Decoder type: exact Log MAP"<<endl;
  else if(decoder_type==CM_APPROX_LOG_MAP) fs<<"!  Decoder type: approximated Log MAP"<<endl;
  else if(decoder_type==CM_MAX_LOG_MAP)    fs<<"!  Decoder type: maximum Log MAP"<<endl;
  
  if(mode==TCM || mode==TTCM){
    fs<<"!  Gen_poly = "<<tcm->GenPoly<<endl;
  }
  else{
    int i,j;
    if(bicm->puncture_code == OFF){
      for(i=0;i<k;i++){
	fs<<"!  Gen_poly["<<i<<"] ";
	fs<<bicm->GenPoly.get_row(i)<<endl;
	//for(j=0;j<n;j++) fs<<bicm->GenPoly(i,j)<<" "; fs<<endl;
      }
    }
    else{
      fs<<"!  Puncturing Pattern = ";
      for(j=0; j<k; j++) fs<<(bicm->puncture[j]&1)<<" ";
      fs<<endl<<"!                       ";
      for(j=0; j<k; j++) fs<<((bicm->puncture[j]>>1) & 1)<<" ";
      fs<<endl;
      
      fs<<"!  Gen_poly[0] = ";
      for(j=0;j<2;j++) fs<<bicm->GenPoly(0,j)<<" ";
      fs<<endl;
    }
  }
  
  fs<<"!  Code termination: "<<(Terminated==ON ? "ON":"OFF")<<endl;
  fs<<"!  Block length: output coded (modulated) symbol ("<<no_of_symbols<<"), input data symbol ("<<no_of_info_symbols<<")"<<endl;
  
  fs<<"!  Modulation: "<<modulation->number_of_levels<<"-"<<(modulation_type==CPSK ? "PSK": "QAM")<<" "<<(modulation->mode==SP ? "Set Partition":"GRAY")<<endl;
  
  if(mode==TTCM)                 
    fs<<"!  Turbo interleaver  : 1 odd-even separated random symbol interleaver of length "<<no_of_symbols<<": Intlv.oes."<<no_of_symbols<<endl;

  if(mode==BICM || mode==BICMID) 
    fs<<"!  Channel interleaver: "<<n<<" parallel random bit interleavers of length "<<no_of_symbols<<": Bit[0-"<<n<<"]Intlv."<<no_of_symbols<<endl;

  if(mode==TCM || mode==TTCM){
    if(interleaver_mode==OFF)    fs<<"!  Channel interleaver: OFF"<<endl;
    if(interleaver_mode==ON)     fs<<"!  Channel interleaver: 1 random symbol interleaver of length "<<no_of_symbols<<": Intlv."<<no_of_symbols<<endl;
  }
  if(interleaver_mode==IQ_INTLV) fs<<"!  Channel interleaver: 2 random IQ interleavers of length "<<no_of_symbols<<": Intlv[I,Q]."<<no_of_symbols<<endl;
  
  fs<<"!"<<endl;
}

void CM::print_coding_tables(FILE *fp)
{
  int i, j;
  
  if(mode==TCM || mode==TTCM){
    fprintf(fp, "CurrentState Branch : Codeword NextState PreviousState\n"); 
      for(i=0;i<tcm->NoStates;i++){
	  for(j=0;j<tcm->NoBranches;j++){
	      fprintf(fp, "%d %d : %d %d %d", 
		      i, j, tcm->Lb(i,j), tcm->Ns(i,j), tcm->Ps(i,j));
	      fprintf(fp, "\n");
          }
      }
      //cout << tcm->Lb << endl;
      //cout << tcm->Ns << endl;
      //cout << tcm->Ps << endl;
  }
  else if(mode==BICM || mode==BICMID)
  {
      int S,M;
      if(bicm->puncture_code==OFF) {S=bicm->NoStates; M=bicm->NoBranches;}
      else                         {S=bicm->NoStates; M=2;}

      fprintf(fp, "CurrentState Branch : Codeword NextState PreviousState\n");
      
      for(i=0;i<S;i++){
	  for(j=0;j<M;j++){
	    if(bicm->Ps(i,j)==NO_LINK)
	      fprintf(fp, "%d %d : %d %d NO_LINK", 
		      i, j, bicm->Lb(i,j), bicm->Ns(i,j));
	    else
	      fprintf(fp, "%d %d : %d %d %d", 
		      i, j, bicm->Lb(i,j), bicm->Ns(i,j), bicm->Ps(i,j));
	    fprintf(fp, "\n");
          }
      }
      //cout << bicm->Lb << endl;
      //cout << bicm->Ns << endl;
      //cout << bicm->Ps << endl;
  }
}

void CM::print_coding_tables(fstream &fs)
{
  int i, j;
  
  if(mode==TCM || mode==TTCM){
      fs<<"CurrentState Branch : Codeword NextState PreviousState"<<endl; 
      for(i=0;i<tcm->NoStates;i++){
	  for(j=0;j<tcm->NoBranches;j++){
	    fs<<i<<" "<<j<<" : "<<tcm->Lb(i,j)<<" "<<tcm->Ns(i,j)<<" "<<tcm->Ps(i,j);
	    fs<<endl;
          }
      }
  }
  else if(mode==BICM || mode==BICMID)
  {
      int S,M;
      if(bicm->puncture_code==OFF) {S=bicm->NoStates; M=bicm->NoBranches;}
      else                         {S=bicm->NoStates; M=2;}

      fs<< "CurrentState Branch : Codeword NextState PreviousState"<<endl; 
      
      for(i=0;i<S;i++){
	  for(j=0;j<M;j++){
	    if(bicm->Ps(i,j)==NO_LINK)
	      fs<<i<<" "<<j<<" : "<<bicm->Lb(i,j)<<" "<<bicm->Ns(i,j)<<" NO_LINK";
	    else
	      fs<<i<<" "<<j<<" : "<<bicm->Lb(i,j)<<" "<<bicm->Ns(i,j)<<" "<<bicm->Ps(i,j);
	    
	    fs<<endl;
          }
      }
  }
}

int CM::encode( bvec b_Input_bits, ivec &i_Output_symbols )
{
  ivec i_Input_bits, i_Input_symbols;

  if(b_Input_bits.length()!=no_of_info_bits){ 
    printf("b_Input_bits.length()=%d no_of_info_bits=%d\n",b_Input_bits.length(), no_of_info_bits);
    s_error("CM::encode: make sure that b_Input_bits.length()==no_of_info_bits ");
  }
  
  i_Input_bits    = to_ivec(b_Input_bits);

  if(Terminated==ON){ 
    /* Insert tail bits */
    ivec tail_bits;
    tail_bits.set_length(no_of_tail_bits, false);
    tail_bits.zeros();
    i_Input_bits = concat(i_Input_bits,tail_bits); 
    //tail_bits.~ivec();
  }

  i_Input_symbols = bits_seq_to_symbol(k, no_of_symbols, i_Input_bits); 

#ifdef debug_cm  
  cout << "i_Input_bits   (" << i_Input_bits.length() <<")= " << i_Input_bits << endl;
  cout << "i_Input_symbols(" << i_Input_symbols.length() << ")= " << i_Input_symbols << endl;
#endif
  
  switch (mode) 
  {
    case TCM:
      {
	ivec Output_symbols;
	Output_symbols.set_size(no_of_symbols, false);
	
	TCMEnc( i_Input_symbols, Output_symbols); 
	
	if(interleaver->mode==ON) intlvi(Output_symbols, i_Output_symbols, no_of_symbols, interleaver->table);
	else i_Output_symbols = Output_symbols;
	
	//Output_symbols.~ivec();
      }
      break;
    case TTCM:
      {
	ivec Output_symbols;
	Output_symbols.set_size(no_of_symbols, false);
	
	TurboTCMEnc( i_Input_symbols, Output_symbols);
	
	if(interleaver->mode==ON) intlvi(Output_symbols, i_Output_symbols, no_of_symbols, interleaver->table);
	else i_Output_symbols = Output_symbols;
	
	//Output_symbols.~ivec();
      }
      break;
    case BICM:
    case BICMID:
      { 
	imat bits_block, bits_block_i;
	int i;
	int N=bicm->bit_interleaver[0]->length;
	
	if(bicm->k==5)
	  BICMEnc_Puncture(i_Input_symbols, i_Output_symbols);
	else
	  BICMEnc(i_Input_symbols, i_Output_symbols);      

	bits_block.set_size(bicm->n, N);
	bits_block_i.set_size(bicm->n, N);

	symbol_to_bits(bicm->n, N, i_Output_symbols, bits_block);	
	for(i=0; i<bicm->n; i++)
	  bit_intlvi(bits_block, bits_block_i, N, bicm->bit_interleaver[i]->table, i );	
	bits_to_symbol(bicm->n, N, bits_block_i, i_Output_symbols);
	
	//bits_block.~imat();
	//bits_block_i.~imat();
	break;
      }
    default:
      s_error("error in CM.mode");
      break;
  }

  //i_Input_bits.~ivec();
  //i_Input_symbols.~ivec();

#ifdef debug_cm  
  cout << "CM.encoded symbols:" << endl << i_Output_symbols<<endl; 
#endif

  return 0;
}

int CM::encode_bits( bvec b_Input_bits, ivec &i_Output_bits )
{
  ivec i_Output_symbols(no_of_symbols);
  
  encode(b_Input_bits, i_Output_symbols);
  i_Output_bits = symbol_to_bits_seq(n, no_of_symbols, i_Output_symbols);

  //b_Output_bits = to_bvec( symbol_to_bits_seq(n, no_of_symbols, i_Output_symbols) );

  return 0;
}

void CM::TCMEnc( ivec Input, ivec &Output)
{  
      int i, m, s;
      //int N = no_of_symbols;
      int N = Input.length();
      
      for(i=s=0; i<N; i++){
          m = Input[i];
          Output[i] = tcm->Lb(s,m); /*Lb = codeword table*/
          s = tcm->Ns(s,m);         /*Ns = next state table*/
      }
      //cout << "CM.TCMEnc:" << endl << Output<<endl; 
}

void CM::BICMEnc( ivec Input, ivec &Output)
{  
      int i, m, s;
      //int N = no_of_symbols;
      int N = Input.length();
      
      for(i=s=0; i<N; i++){
          m = Input[i];
          Output[i] = bicm->Lb(s,m); /*Lb = codeword table*/
          s = bicm->Ns(s,m);         /*Ns = next state table*/
      }
}

void CM::BICMEnc_Puncture( ivec Input, ivec &Output)
{  
      int i, m, s, p;
      ivec pInput, pOutput, p2Output;
      //int N = no_of_symbols;
      int N = Input.length();
      
      pInput  .set_size(N*bicm->k);
      pOutput .set_size(N*bicm->k);
      p2Output.set_size(N*(bicm->k+1));

      /* split Input */
      pInput=symbol_to_bits_seq(bicm->k, N, Input);
      
      /* 1/2-rate encoding */
      for(i=s=0; i<bicm->k*N; i++){
          m = pInput[i];
          pOutput[i] = bicm->Lb(s,m); /*Lb = codeword table*/
          s = bicm->Ns(s,m);          /*Ns = next state table*/
      }
      
      /* puncture output */
      for(s=i=0; i<bicm->k*N;){
          for(p=0; p<bicm->k; p++, i++){
              for(m=0; m<2; m++){
                  if( (bicm->puncture[p]>>m) & 1 ){
                    /*printf("%d=%d (%d=%d)  ", s, (pOutput[i]>>m) & 1, i, pOutput[i] );*/
                    p2Output[s++] = (pOutput[i]>>m) & 1; 
                  }               
              } 
          }
      }
      
      /* combine pOutput */
      Output = bits_seq_to_symbol(bicm->n, N, p2Output);

      //pInput.~ivec();
      //pOutput.~ivec();
      //p2Output.~ivec();
}

void CM::TurboTCMEnc( ivec Input, ivec &Output)
{
     ivec v1, v2, v2_i, Input_i;
     int i, j, m, s;
     //int N=no_of_symbols;
     int N=Input.length();

     v1.set_length(N,false);
     v2_i.set_length(N,false);
     v2.set_length(N,false);
     Input_i.set_length(N,false);
      
     if(Terminated==ON)
     {
         /* Insert known symbols */
         for ( i=0; i<tcm->L; i++)
         {
             j=N-1;
             while( j > tcm->last[i] )
             {
                 Input[j] = Input[j-1];
                 j--;
             }
             Input[ tcm->last[i] ] = 0;
         }
         for ( i=0; i<tcm->L; i++ ) Input[N-1-i] = 0;
     }

     
     /* coder 1 */
     for(i=s=0; i<N; i++){
         m = Input[i];
         v1[i] = tcm->Lb(s,m);    /*Lb = codeword table*/
         s = tcm->Ns(s,m);        /*Ns = next state table*/
     }
         
     intlvi(Input, Input_i, N, ttcm->interleaver->table);

     /* coder 2 */
     for(i=s=0; i<N; i++){
         m = Input_i[i];
         v2_i[i] = tcm->Lb(s,m);    /*Lb = codeword table*/
         s = tcm->Ns(s,m);        /*Ns = next state table*/
     }

     de_intlvi(v2_i, v2, N, ttcm->interleaver->table);
         
     for(i=0; i<N; i++){
         if(i%2 == 0)
             Output[i] = v1[i];
         else
             Output[i] = v2[i];
     }

     //v1.~ivec();
     //v2_i.~ivec();
     //v2.~ivec();
     //Input_i.~ivec();
}

void CM::decode_symbol(mat Apo, ivec &symbols, int N, int M)
{
    int k,i,m;
    double max;

    for(k=0; k<N; k++) 
    {
        i=0; max=Apo(k,0);
        for(m=1; m<M; m++)
        {    
            if(Apo(k,m) > max)
            {
                max = Apo(k,m);
                i = m;
            }
        }
        symbols[k] = i;
    }

#ifdef debug_cm
    cout<<"decoded symbols" << symbols<<endl;
#endif

}

vec CM::decode(bvec &b_decoded_bits, vec cm_apr_databits_llr)
{
        int N, M ,S;
        int j, m, i;
	ivec i_decoded_symbols;
	ivec i_decoded_bits;

	mat *Apr_data_bit;

        N  = no_of_symbols;  
        M  = 1<<k;
        S  = 1<<L;

	//Apr_data_bit  = (mat *)calloc(sizeof(mat), k);
	Apr_data_bit  = new mat[k];
	for(i=0;i<k;i++) Apr_data_bit[i]  = mat(N, 2);
	//for(i=0;i<k;i++) Apr_data_bit[i].clear();	
	
	if(Terminated==ON){
	  /* Insert tail bits */
	  vec tail_bits;
	  
	  tail_bits.set_length(no_of_tail_bits, false);
	  for(i=0;i<no_of_tail_bits;i++) tail_bits[i]=MINF;
	  cm_apr_databits_llr = concat(cm_apr_databits_llr, tail_bits);
	  
	  //tail_bits.~vec();
	}
	
	LLR_to_Prob(N, k, cm_apr_databits_llr, Apr_data_bit);
	//cout<<"Apr: "; for(i=N-10;i<N;i++) cout<<"("<<i<<") "<<Apr_data_bit[0](i,0)<<" "<<Apr_data_bit[0](i,1)<<" "<<endl;
	//cout<<cm_apr_databits_llr<<endl; getchar();
	
	if(b_decoded_bits.length()!=no_of_info_bits){ 
	  printf("b_decoded_bits.length()=%d no_of_info_bits=%d\n",b_decoded_bits.length(), no_of_info_bits);
	  s_error("CM::decode: make sure that b_decoded_bits.length()==no_of_info_bits ");
	}
	
	//cout<<Apr_data_bit[0]<<endl; getchar();

	if(interleaver->mode==IQ_INTLV){
	  mat i_PrI, i_PrQ;
	  i_PrI = mat(PrI.rows(), PrI.cols());
	  i_PrQ = mat(PrQ.rows(), PrQ.cols());
	  
	  //cout<<PrI<<endl; getchar();  
	  de_intlvd_2d( PrI, i_PrI, PrI.rows(), interleaver->i_table);	  
	  de_intlvd_2d( PrQ, i_PrQ, PrQ.rows(), interleaver->q_table);
	  compute_Pr_from_PrIQ(Pr, i_PrI, i_PrQ);
	  
	  //i_PrI.~mat(); i_PrQ.~mat();
 	}
	
        switch (mode) 
        {
            case TCM:
            {
	        mat i_Pr;
		//i_Pr.operator=(Pr);
		i_Pr=Pr;
		
		if(interleaver->mode==ON)
		  de_intlvd_2d( Pr, i_Pr, Pr.rows(), interleaver->table);
		
		for(j=0; j<N; j++){
		  for(m=0; m<M; m++){		    
		    map->Apr(j,m) = 0.0;
		    for(i=0;i<tcm->k;i++){
		      map->Apr(j,m) += Apr_data_bit[i](j, (m>>i)&1 );
		    }
		    //map->Apr(j,m) = -log(M);
		    
		    for(i=0; i<S; i++)
		      map->Ip1[j](i,m) = i_Pr(j,tcm->Lb(i,m) );
		  }
		}
		//cout<<map->Apr.get_rows(0,N-1)<<endl; getchar();
		
		/* take into account known symbols */
		if(Terminated==ON){
		  for (j=0; j<tcm->L; j++ ){
		    map->Apr(N-1-j,0) = 0;
		    for( m=1; m<M; m++ ) map->Apr(N-1-j,m) = MINF;
		  }
		}
                
		log_mapdec(N, M, S, tcm->Ps, tcm->Ns, tcm->Lb, 
			   map->Apr, map->Ip1, map->Apo, OPr);
		
		// interleave coded symbol's APO for feedback to equaliser
		if(interleaver->mode==ON){
		  intlvd_2d( OPr, i_Pr, OPr.rows(), interleaver->table);		
		  OPr = i_Pr;
		}
		
                // Final decoding
		i_decoded_symbols.set_length(N,false);
                decode_symbol(map->Apo, i_decoded_symbols, N, tcm->NoBranches);
		
                i_decoded_bits = symbol_to_bits_seq(tcm->k, i_decoded_symbols.length(), i_decoded_symbols); 
		
		//b_decoded_bits = to_bvec(i_decoded_bits.mid(0,no_of_info_bits));
		b_decoded_bits = to_bvec(i_decoded_bits.left(no_of_info_bits));

		//i_Pr.~mat();
                break;
            }
            case TTCM:
	    {
	        int j;
	        mat i_Pr;
		i_Pr=Pr;
		
		//if(interleaver->mode!=IQ_INTLV)
		if(interleaver->mode==ON)
		  de_intlvd_2d( Pr, i_Pr, interleaver->length, interleaver->table);
		
		for(j=0; j<N; j++){
		  /* init data for MAP dec */
		  if ( j%2 == 0 ){
		    
		    for(m=0; m<M; m++){
		      /* apri info */
		      map->Apr(j,m) = 0.0;
		      for(i=0;i<tcm->k;i++){
			map->Apr(j,m) += Apr_data_bit[i](j, (m>>i)&1 );
		      }
		      //map->Apr(j,m) = -log(M);
		      
		      /* parity info */
		      for( i=0; i<S; i++) {
			map->Ip1[j](i,m) = i_Pr(j,tcm->Lb(i,m) );
			map->Ip2[j](i,m) = 0; //punctured, all possible
		      }  
		    }
		  }
		  else{
		    for(m=0; m<M; m++){
		      /* apri for 2nd decoder */
		      map->Apr(j,m) = 0.0;
		      for(i=0;i<tcm->k;i++){
			map->Apr(j,m) += Apr_data_bit[i](j, (m>>i)&1 );
		      }
		      //map->Apr(j,m) = -log(M);
		      
		      /* parity info */
		      for( i=0; i<S; i++) {
			map->Ip2[j](i,m) = i_Pr(j,tcm->Lb(i,m) );
			map->Ip1[j](i,m) = 0;
		      }
		    }
		  }
		}
		//cout<<i_Pr<<endl; getchar();
		//cout<<map->Ip1[0]<<map->Ip2[1]<<endl; getchar();
		//cout<<map->Apr.get_rows(0,N-1)<<endl; getchar();
		
		if(Terminated==ON){
                    /* take into account known (terminated) symbols */
		    for ( i=0; i<tcm->L; i++)
		    {
			j=N-1;
			while( j > tcm->last[i] )
			{
			    for( m=0; m<M; m++ ) map->Apr(j,m) =  map->Apr(j-1,m);
			    j--;
			}
			map->Apr( tcm->last[i] ,0) = 0;
			for( m=1; m<M; m++ ) map->Apr(tcm->last[i] ,m) = MINF;
		    }
		    for ( i=0; i<tcm->L; i++ ){
		      map->Apr(N-1-i, 0) = 0;
		      for( m=1; m<M; m++ ) map->Apr(N-1-i, m) = MINF;
		    }
		    /*
		      for (j=0; j<tcm->L; j++ )
		      {
		      map->Apr(N-1-j,0) = 0;
		      for( m=1; m<M; m++ ) map->Apr(N-1-j,m) = MINF;
		      
		      map->Apr( tcm->last[j] ,0) = 0;
		      for( m=1; m<M; m++ ) map->Apr(tcm->last[j],m) = MINF;
		      }
		    */
		}		
		
		//log_TTCM_dec(N, M, S, tcm->Ps, tcm->Ns, tcm->Lb, 
		log_TTCM_dec_apr(N, M, S, tcm->Ps, tcm->Ns, tcm->Lb, 
			     map->Apr, map->Ip1, map->Ip2, map->Apo, OPr, OPr2);
		
		// interleave coded symbol's APO for feedback to equaliser
		if(interleaver->mode==ON){
		  intlvd_2d( OPr, i_Pr, OPr.rows(), interleaver->table);		
		  OPr = i_Pr;
		}
		
		// Final decoding
		i_decoded_symbols.set_length(N,false);
		decode_symbol(map->Apo, i_decoded_symbols, N, tcm->NoBranches);
		
		if(Terminated==ON){
		    // remove known symbols
                    for ( j=tcm->L-1; j>=0; j-- )
                    {
                        i=tcm->last[j];
                        while( i<N-1 )
                        {
                            i_decoded_symbols[i] = i_decoded_symbols[i+1];

			    for(m=0; m<M; m++){
			      map->Apo(i,m) = map->Apo(i+1,m);
			      map->Extr(i,m) = map->Extr(i+1,m);
			    }
                            i++;
                        }
                    }
                    for (j=N - 2*tcm->L; j<N; j++){
		      i_decoded_symbols[j]=0;
		      
		      map->Apo(j,0) = 0;  
		      map->Extr(j,0) = 0;
		      for(m=1; m<M; m++){
			map->Apo(j,m) = MINF;
			map->Extr(j,m) = MINF;
		      }
		    }
		}
		
                i_decoded_bits = symbol_to_bits_seq(tcm->k, i_decoded_symbols.length(), i_decoded_symbols); 
		b_decoded_bits = to_bvec(i_decoded_bits.left(no_of_info_bits));
		
		//i_Pr.~mat();
		break;
	    }
            case BICM:
	    case BICMID:
	    {
		mat *Apr_coded_bit_i;
		
		//Apr_coded_bit_i = (mat *)calloc(sizeof(mat), bicm->n);
		Apr_coded_bit_i = new mat[bicm->n];
		for(i=0;i<bicm->n;i++) Apr_coded_bit_i[i] = mat(N, 2); 
		
		Pr_to_BitPr_log(N, bicm->n, Pr, siso->Apr_coded_bit, 0);
		
		/* De-interleave the extrinsic Apo from modulator (channel deinterleaver) */
		for(i=0; i<bicm->n; i++)
		  bit_de_intlvm_3d( siso->Apr_coded_bit, Apr_coded_bit_i, N, 2,
				    bicm->bit_interleaver[i]->table, i );
		
		if(bicm->k==5)
		  BICM_ID_dec_log_pun(N, bicm->k, bicm->NoBranches, bicm->NoStates, bicm->Ps, bicm->Ns, bicm->Lb, 
				      Apr_coded_bit_i, Apr_data_bit, siso->Apo_coded_bit, siso->Apo_data_bit, 
				      siso->Apo_dataword, Terminated);
		else
		  BICM_ID_dec_log    (N, bicm->k, bicm->NoBranches, bicm->NoStates, bicm->Ps, bicm->Ns, bicm->Lb, 
				      Apr_coded_bit_i, Apr_data_bit, siso->Apo_coded_bit, siso->Apo_data_bit, 
				      siso->Apo_dataword, Terminated);
		
		// Final decoding
		i_decoded_symbols.set_length(N,false);
		decode_symbol(siso->Apo_dataword, i_decoded_symbols, N, bicm->NoBranches);
		
                i_decoded_bits = symbol_to_bits_seq(bicm->k, i_decoded_symbols.length(), i_decoded_symbols);		
		b_decoded_bits = to_bvec(i_decoded_bits.left(no_of_info_bits));
		
		//for(i=0;i<bicm->n;i++) Apr_coded_bit_i[i].~mat(); 
		//free(Apr_coded_bit_i);
		delete [] Apr_coded_bit_i;
		break;
	    }
            default:
	        s_error("error in CM.mode");
                break;
	}

#ifdef debug_cm	
	cout<<i_decoded_bits <<endl;
	cout<<b_decoded_bits <<endl;
#endif
	
	if(Terminated==ON){
	  /* Delete tail bits */
	  cm_apr_databits_llr = cm_apr_databits_llr.left(cm_apr_databits_llr.length()-no_of_tail_bits);
	}
	//cout<<cm_apr_databits_llr<<endl; getchar();
	
	//i_decoded_symbols.~ivec();
	//i_decoded_bits.~ivec();		
	
	//for(i=0;i<k;i++) Apr_data_bit[i].~mat(); free(Apr_data_bit);	
	delete[] Apr_data_bit;
	
	return(get_data_llr()); 
}


int CM::decode(bvec &b_decoded_bits)
{
        int N, M ,S;
        int j, m, i;
	ivec i_decoded_symbols;
	ivec i_decoded_bits;

	mat *Apr_data_bit;

        N  = no_of_symbols;  
        M  = 1<<k;
        S  = 1<<L;
	
	//Apr_data_bit  = (mat *)calloc(sizeof(mat), k);
	Apr_data_bit  = new mat[k];
	for(i=0;i<k;i++) Apr_data_bit[i]  = mat(N, 2);
	for(i=0;i<k;i++) Apr_data_bit[i].clear();

	if(b_decoded_bits.length()!=no_of_info_bits){ 
	  printf("b_decoded_bits.length()=%d no_of_info_bits=%d\n",b_decoded_bits.length(), no_of_info_bits);
	  s_error("CM::decode: make sure that b_decoded_bits.length()==no_of_info_bits ");
	}
	
	if(interleaver->mode==IQ_INTLV){
	  mat i_PrI, i_PrQ;
	  i_PrI = mat(PrI.rows(), PrI.cols());
	  i_PrQ = mat(PrQ.rows(), PrQ.cols());
	  
	  de_intlvd_2d( PrI, i_PrI, PrI.rows(), interleaver->i_table);	  
	  de_intlvd_2d( PrQ, i_PrQ, PrQ.rows(), interleaver->q_table);
	  compute_Pr_from_PrIQ(Pr, i_PrI, i_PrQ);
	  
	  //i_PrI.~mat(); i_PrQ.~mat();
 	}
	
        switch (mode) 
        {
            case TCM:
            {
	        mat i_Pr;
		//i_Pr.operator=(Pr);
		i_Pr=Pr;
		
		if(interleaver->mode==ON)
		  de_intlvd_2d( Pr, i_Pr, Pr.rows(), interleaver->table);
		
		for(j=0; j<N; j++){
		  for(m=0; m<M; m++){
		    map->Apr(j,m) = -log(M);
		    for(i=0; i<S; i++)
		      map->Ip1[j](i,m) = i_Pr(j,tcm->Lb(i,m) );
		  }
		}
                                        
		/* take into account known symbols */
		if(Terminated==ON){
		  for (j=0; j<tcm->L; j++ ){
		    map->Apr(N-1-j,0) = 0;
		    for( m=1; m<M; m++ ) map->Apr(N-1-j,m) = MINF;
		  }
		}
                
		log_mapdec(N, M, S, tcm->Ps, tcm->Ns, tcm->Lb, 
			   map->Apr, map->Ip1, map->Apo, OPr);

		// interleave coded symbol's APO for feedback to equaliser
		if(interleaver->mode==ON){
		  intlvd_2d( OPr, i_Pr, OPr.rows(), interleaver->table);		
		  OPr = i_Pr;
		}

                // Final decoding
		i_decoded_symbols.set_length(N,false);
                decode_symbol(map->Apo, i_decoded_symbols, N, tcm->NoBranches);
		
                i_decoded_bits = symbol_to_bits_seq(tcm->k, i_decoded_symbols.length(), i_decoded_symbols); 
		
		//b_decoded_bits = to_bvec(i_decoded_bits.mid(0,no_of_info_bits));
		b_decoded_bits = to_bvec(i_decoded_bits.left(no_of_info_bits));

		//i_Pr.~mat();
                break;
            }
            case TTCM:
	    {
	        int k;
	        mat i_Pr;
		//i_Pr.operator=(Pr);
		i_Pr=Pr;
		
		//if(interleaver->mode!=IQ_INTLV)
		if(interleaver->mode==ON)
		  de_intlvd_2d( Pr, i_Pr, interleaver->length, interleaver->table);
		
		for(k=0; k<N; k++){
		  /* init data for MAP dec */
		  if ( k%2 == 0 ){
		    for(m=0; m<M; m++){
		      /* apri info */
		      map->Apr(k,m) = -log(M);
		      
		      /* parity info */
		      for( i=0; i<S; i++) {
			map->Ip1[k](i,m) = i_Pr(k,tcm->Lb(i,m) );
			map->Ip2[k](i,m) = 0; //punctured, all possible
		      }  
		    }
		  }
		  else{
		    for(m=0; m<M; m++)
		      {
			/* apri for 2nd decoder */
			map->Apr(k,m) = -log(M);
			
			/* parity info */
			for( i=0; i<S; i++) {
			  map->Ip2[k](i,m) = i_Pr(k,tcm->Lb(i,m) );
			  map->Ip1[k](i,m) = 0;
			}
		      }
		  }
		}
		//cout<<Pr<<endl; getchar();
		//cout<<map->Ip1[0]<<map->Ip2[1]<<endl; getchar();

		if(Terminated==ON)
                    /* take into account known (terminated) symbols */
                        for (k=0; k<tcm->L; k++ )
                        {
                            map->Apr(N-1-k,0) = 0;
                            for( m=1; m<M; m++ ) map->Apr(N-1-k,m) = MINF;
                            map->Apr( tcm->last[k] ,0) = 0;
                            for( m=1; m<M; m++ ) map->Apr(tcm->last[k],m) = MINF;
                        }
                    
		log_TTCM_dec(N, M, S, tcm->Ps, tcm->Ns, tcm->Lb, 
			     map->Apr, map->Ip1, map->Ip2, map->Apo, OPr, OPr2);
		
		// interleave coded symbol's APO for feedback to equaliser
		if(interleaver->mode==ON){
		  intlvd_2d( OPr, i_Pr, OPr.rows(), interleaver->table);		
		  OPr = i_Pr;
		}
		
		// Final decoding
		i_decoded_symbols.set_length(N,false);
		decode_symbol(map->Apo, i_decoded_symbols, N, tcm->NoBranches);
		
		if(Terminated==ON){
		    // remove known symbols
                    for ( k=tcm->L-1; k>=0; k-- )
                    {
                        i=tcm->last[k];
                        while( i<N-1 )
                        {
                            i_decoded_symbols[i] = i_decoded_symbols[i+1];

			    for(m=0; m<M; m++) 
			      map->Apo(i,m) = map->Apo(i+1,m);

                            i++;
                        }
                    }
                    for (k=N - 2*tcm->L; k<N; k++) i_decoded_symbols[k]=0;
		}

                i_decoded_bits = symbol_to_bits_seq(tcm->k, i_decoded_symbols.length(), i_decoded_symbols); 
		b_decoded_bits = to_bvec(i_decoded_bits.left(no_of_info_bits));
		
		//i_Pr.~mat();
		break;
	    }
            case BICM:
	    case BICMID:
	      {
		mat *Apr_coded_bit_i;
		
		//Apr_coded_bit_i = (mat *)calloc(sizeof(mat), bicm->n);
		Apr_coded_bit_i = new mat[bicm->n];
		for(i=0;i<bicm->n;i++) Apr_coded_bit_i[i] = mat(N, 2); 
		
		Pr_to_BitPr_log(N, bicm->n, Pr, siso->Apr_coded_bit, 0);
		
		/* De-interleave the extrinsic Apo from modulator (channel deinterleaver) */
		for(i=0; i<bicm->n; i++)
		  bit_de_intlvm_3d( siso->Apr_coded_bit, Apr_coded_bit_i, N, 2,
				    bicm->bit_interleaver[i]->table, i );
		
		if(bicm->k==5)
		  BICM_ID_dec_log_pun(N, bicm->k, bicm->NoBranches, bicm->NoStates, bicm->Ps, bicm->Ns, bicm->Lb, 
				      Apr_coded_bit_i, Apr_data_bit, siso->Apo_coded_bit, siso->Apo_data_bit, 
				      siso->Apo_dataword, Terminated);
		else
		  BICM_ID_dec_log    (N, bicm->k, bicm->NoBranches, bicm->NoStates, bicm->Ps, bicm->Ns, bicm->Lb, 
				      Apr_coded_bit_i, Apr_data_bit, siso->Apo_coded_bit, siso->Apo_data_bit, 
				      siso->Apo_dataword, Terminated);
		
		// Final decoding
		i_decoded_symbols.set_length(N,false);
		decode_symbol(siso->Apo_dataword, i_decoded_symbols, N,  bicm->NoBranches);
		
                i_decoded_bits = symbol_to_bits_seq(bicm->k, i_decoded_symbols.length(), i_decoded_symbols);		
		b_decoded_bits = to_bvec(i_decoded_bits.left(no_of_info_bits));		
		
		//for(i=0;i<bicm->n;i++) Apr_coded_bit_i[i].~mat(); 
		//free(Apr_coded_bit_i);
		delete [] Apr_coded_bit_i;
		break;
	      }
            default:
	        s_error("error in CM.mode");
                break;
	}

#ifdef debug_cm	
	cout<<i_decoded_bits <<endl;
	cout<<b_decoded_bits <<endl;
#endif

	//i_decoded_symbols.~ivec();
	//i_decoded_bits.~ivec();		
	
	//for(i=0;i<k;i++) Apr_data_bit[i].~mat();
	//free(Apr_data_bit);
	delete [] Apr_data_bit;

	return 0;
}

int CM::decode_using_Pr( bvec &b_decoded_bits, mat in_Pr)
{
  if(in_Pr.cols()!=Pr.cols() || in_Pr.rows()!=Pr.rows()){
    printf("incorrect input Probability matrix size c=%d(%d) r=%d(%d)\n",
    in_Pr.cols(),Pr.cols(),in_Pr.rows(),Pr.rows());
    exit(-1);
  }
  
  Pr = in_Pr;
  
  b_decoded_bits.set_length(no_of_info_bits);
  decode(b_decoded_bits);
}

int CM::decode_using_PrIQ( bvec &b_decoded_bits, mat in_PrI, mat in_PrQ)
{
  if(in_PrI.cols()!=PrI.cols() || in_PrI.rows()!=PrI.rows() || in_PrQ.cols()!=PrQ.cols() || in_PrQ.rows()!=PrQ.rows()){
    printf("incorrect input Probability matrix size cI=%d(%d) rI=%d(%d)  cQ=%d(%d) rQ=%d(%d)\n",
    in_PrI.cols(),PrI.cols(),in_PrI.rows(),PrI.rows(), in_PrQ.cols(),PrQ.cols(),in_PrQ.rows(),PrQ.rows());
    exit(-1);
  }
  
  PrI = in_PrI;    
  PrQ = in_PrQ;    
  decode(b_decoded_bits);
}

/*-----LLR <-> Prob----------------*/
void CM::SymProb_to_LLR ( int N, int bps, vec &LLR, mat SymPr )
{
  int i;
  mat *BitBlockPr;

  BitBlockPr  = new mat[bps];
  for(i=0;i<bps;i++) BitBlockPr[i] = mat(N, 2);

  Pr_to_BitPr_log(N, bps, SymPr, BitBlockPr, 0);
  Prob_to_LLR (N, bps, LLR, BitBlockPr);

  delete[] BitBlockPr;
}

void CM::LLR_to_SymProb ( int N, int bps, vec LLR, mat & SymPr )
{
  int i;
  mat *BitBlockPr;

  BitBlockPr  = new mat[bps];
  for(i=0;i<bps;i++) BitBlockPr[i] = mat(N, 2);
  
  LLR_to_Prob(N, bps, LLR, BitBlockPr);
  BitPr_to_Pr_log(N, bps, SymPr, BitBlockPr, 0);

  delete[] BitBlockPr;	
}

void CM::Prob_to_LLR ( int N, int bps, vec &LLR, mat* BitPr )
{
  int k, i;

  for ( k = 0; k < N; k++ )
    for ( i = 0; i < bps; i++ )
        LLR[k * bps + i] = BitPr[i](k,1) - BitPr[i](k,0); 
}

void CM::LLR_to_Prob ( int N, int bps, vec LLR, mat *& BitPr )
{
  int k, i;
  double max;
  
  if((N*bps)!=LLR.length()) it_error("N*bps!=LLR.length() in LLR_to_Prob (cm.cpp)");

  for ( k = 0; k < N; k++ )
  {
    for ( i = 0; i < bps; i++ )
    {
      BitPr[i](k,0) = - jacolog_1 ( 0.0, LLR[k * bps + i] );
      BitPr[i](k,1) = - jacolog_1 ( 0.0, - LLR[k * bps + i] );
      //max = jacolog_1 (BitPr[i](k,0), BitPr[i](k,1));      
      //BitPr[i](k,0) -= max;
      //BitPr[i](k,1) -= max;
      //if(k>N-6) printf("k%d i%d %e %e llr=%e  ", k, i, BitPr[i](k,0), BitPr[i](k,1), LLR[k * bps + i]);
    }
  }

}

vec CM::get_data_llr(void)
{
  vec data_llr;
  data_llr.set_size(no_of_info_bits,false);
  
  //map->Apo;
  if(mode==BICM || mode==BICMID){
    // the aposteriori value 
    Prob_to_LLR ( no_of_info_symbols, bicm->k, data_llr, siso->Apo_data_bit);
  }
  else if(mode==TCM || mode==TTCM){
    int i, N;
    mat *Apo_data_bit;
    
    N=no_of_info_symbols; 
    
    Apo_data_bit  = new mat[tcm->k];
    for(i=0;i<tcm->k;i++) Apo_data_bit[i]  = mat(N, 2); 
    
    // the aposteriori value 
    Pr_to_BitPr_log(N, tcm->k, map->Apo, Apo_data_bit, 0);    
    Prob_to_LLR ( no_of_info_symbols, tcm->k, data_llr, Apo_data_bit);

    delete [] Apo_data_bit;
  }
  else
    it_error("invalid CM mode at CM::get_data_llr()");
  
  //cout<<data_llr<<endl<<data_llr.size()<<endl; getchar();
  return data_llr;
}

vec CM::get_data_extr_llr(void)
{
  vec data_llr;
  data_llr.set_size(no_of_info_bits,false);
  
  //map->Apo;
  if(mode==BICM || mode==BICMID){
    // the aposteriori value 
    Prob_to_LLR ( no_of_info_symbols, bicm->k, data_llr, siso->Apo_data_bit);
  }
  else if(mode==TCM || mode==TTCM){
    int i, N;
    mat *Apo_data_bit;
    
    N=no_of_info_symbols; 
    
    Apo_data_bit  = new mat[tcm->k];
    for(i=0;i<tcm->k;i++) Apo_data_bit[i]  = mat(N, 2); 
    
    // the aposteriori value 
    Pr_to_BitPr_log(N, tcm->k, map->Extr, Apo_data_bit, 0);    
    Prob_to_LLR ( no_of_info_symbols, tcm->k, data_llr, Apo_data_bit);
    
    delete [] Apo_data_bit;
  }
  else
    it_error("invalid CM mode at CM::get_data_llr()");
  
  //cout<<data_llr<<endl<<data_llr.size()<<endl; getchar();
  return data_llr;
}

void CM::log_mapdec(int N, int M, int S, imat Ps, imat Ns, imat Lb,
		    mat Apr, mat *Ip, mat &Apo, mat &OPr)
{
        int i, k, m;
        mat alpha, beta;  
        double max=MINF, abc, max_OPr=MINF;
        
        alpha = mat(N+1,S+1);
        beta  = mat(N+1,S+1);
        
        /* compute alpha */
        for(i=0; i<S; i++) alpha(0,i)=MINF;
        alpha(0,0)=0.;
        for(k=1; k<=N; k++) 
        {       
                max = MINF;
                for(i=0; i<S; i++)
                {
                        /* init: m=0 and 1 */
		        alpha(k,i) = jacolog( alpha(k-1, Ps(i,0)) + Ip[k-1](Ps(i,0),0) + Apr(k-1,0),
					      alpha(k-1,Ps(i,1)) + Ip[k-1](Ps(i,1),1) + Apr(k-1,1));
                        for( m=2; m<M; m++)
                            alpha(k,i) = jacolog( alpha(k,i), 
						  alpha(k-1,Ps(i,m)) + Ip[k-1](Ps(i,m),m) + Apr(k-1,m));
                              
                        if ( max < alpha(k,i) ) max = alpha(k,i);
                }
                for(i=0; i<S; i++) alpha(k,i) -= max;
        }

        /* compute beta */
        for(i=0; i<S; i++) beta(N,i)=0.;
        for(k=N-1; k>=0; k--) 
        {  
                max = MINF;
                for(i=0; i<S; i++)
                {
                        beta(k,i)= jacolog( beta(k+1,Ns(i,0)) + Ip[k](i,0) + Apr(k,0),
					    beta(k+1,Ns(i,1)) + Ip[k](i,1) + Apr(k,1));
                        for( m=2; m<M; m++) 
                            beta(k,i) = jacolog( beta(k,i), 
						 beta(k+1,Ns(i,m)) + Ip[k](i,m) + Apr(k,m));
                        
                        if ( max < beta(k,i) ) max = beta(k,i);
                }
                for(i=0; i<S; i++) beta(k,i) -= max;
        }

        /* compute apo */
        for(k=0; k<N; k++) 
        {
                max = MINF;
                max_OPr = MINF;
                
                for(m=0; m<2*M; m++) OPr(k,m)=MINF; /* some links are not possible */
                
                for(m=0; m<M; m++)
                {
                        Apo(k,m) = MINF; 
                        for(i=0; i<S; i++) 
                        {
                            abc = alpha(k,Ps(i,m)) + beta(k+1,i) + Ip[k](Ps(i,m),m);
                            Apo(k,m) = jacolog( Apo(k,m), abc );
                            OPr(k, Lb( Ps(i,m), m) ) = jacolog( OPr(k, Lb(Ps(i,m), m) ), abc + Apr(k,m));
                        } 
                        
			//map->Extr(k,m) = Apo(k,m);// mike debug purpose
			
                        Apo(k,m) += Apr(k,m);
                        if ( max < Apo(k,m) ) max = Apo(k,m);

                }
		for(m=0; m<M; m++) Apo(k,m) -= max;
                
                //for(m=0; m<2*M; m++) if(max_OPr < OPr[k][m]) max_OPr = OPr[k][m];
                //for(m=0; m<2*M; m++) OPr[k][m] -= max_OPr;
        }

        /* done */
	//alpha.~mat();
	//beta.~mat();
  return;
}

void CM::log_TTCM_dec_apr(int N, int M, int S, imat Ps, imat Ns, imat Lb,
			  mat Apr, mat *Ip1, mat *Ip2,
			  mat &Apo, mat &OPr, mat &OPr2)
{
        int i, k, m;
	mat Apr_i, Apo_i, OPr2_i;
	mat * Ip2_i;
	mat Apr_d, Apr_d_i;
	
	Apr_i.set_size( N, M, false); 
	Apo_i.set_size( N, M, false);

	OPr2_i.set_size( N, 2*M, false );
	
	//Ip2_i = (mat *) calloc( sizeof(mat), N);
	Ip2_i = new mat[N]; 
	for(i=0; i<N; i++)  Ip2_i[i].set_size( S, M, false );	

	Apr_d.set_size( N, M, false); 
	Apr_d_i.set_size( N, M, false); 
	Apr_d = Apr; 
	Apr.zeros();
	intlvd_2d(Apr_d, Apr_d_i, ttcm->interleaver->length, ttcm->interleaver->table);
	
        /* Turbo decoding */        
        intlvm_3d(Ip2, Ip2_i, ttcm->interleaver->length, ttcm->interleaver->table);
	
        for(i=0; i<ttcm->iterations; i++){
            /* 1st decoder */
	    log_mapdec(N, M, S, Ps, Ns, Lb, Apr+Apr_d, Ip1, Apo, OPr);
            
            for (k=0; k<N; k++) for(m=0; m<M; m++){
	      if( Apr(k,m)+Apr_d(k,m) > MINF ) Apr(k,m) = Apo(k,m) - Apr(k,m) - Apr_d(k,m);
	    }
	    intlvd_2d(Apr, Apr_i, ttcm->interleaver->length, ttcm->interleaver->table);
            
            /* 2nd decoder */
            log_mapdec(N, M, S, Ps, Ns, Lb, Apr_i+Apr_d_i, Ip2_i, Apo_i, OPr2_i); 
            
            if(i==ttcm->iterations-1){
	      // the true extrinsic value has to be calculated here : based on the 2nd decoder's apriori 
	      //for(k=0; k<N; k++) for(m=0; m<M; m++){
	      //if( Apr_d_i(k,m) > MINF ) Apr_i(k,m) = Apo_i(k,m) - Apr_d_i(k,m);
		//if( Apr_i(k,m)+Apr_d_i(k,m) > MINF ) Apr_i(k,m) = Apo_i(k,m) - Apr_d_i(k,m);
		//if( Apr_i(k,m)+Apr_d_i(k,m) > MINF ) Apr_i(k,m) = Apo_i(k,m) - Apr_i(k,m) - Apr_d_i(k,m);
	      //}
	      //de_intlvd_2d(Apr_i, map->Extr, ttcm->interleaver->length, ttcm->interleaver->table);
	      break;
            }
	    
            for(k=0; k<N; k++) for(m=0; m<M; m++){
	      if( Apr_i(k,m)+Apr_d_i(k,m) > MINF ) Apr_i(k,m) = Apo_i(k,m) - Apr_i(k,m) - Apr_d_i(k,m);
	    }
            de_intlvd_2d(Apr_i, Apr, ttcm->interleaver->length, ttcm->interleaver->table);
        }
        
        de_intlvd_2d(Apo_i, Apo, ttcm->interleaver->length, ttcm->interleaver->table);
	/* Apo=real Apo, not extrinsic */
        
        de_intlvd_2d(OPr2_i, OPr2, ttcm->interleaver->length, ttcm->interleaver->table);
        
        for(i=0;i<N;i++)
          if(i%2!=0)
            for(m=0;m<2*M;m++)
              OPr(i,m) = OPr2(i,m);
	
	// free memory
	//for(i=0;i<N;i++) Ip2_i[i].~mat(); free(Ip2_i);
	delete [] Ip2_i;
	//Apr_i.~mat(); 
	//Apo_i.~mat();
	//OPr2_i.~mat();
	//Apr_d.~mat(); 
	//Apr_d_i.~mat();
}

void CM::log_TTCM_dec(int N, int M, int S, imat Ps, imat Ns, imat Lb,
		      mat Apr, mat *Ip1, mat *Ip2,
		      mat &Apo, mat &OPr, mat &OPr2)
{
        int i, k, m;
	mat Apr_i, Apo_i, OPr2_i;
	mat * Ip2_i;
	
	Apr_i.set_size( N, M, false); 
	Apo_i.set_size( N, M, false);
	
	OPr2_i.set_size( N, 2*M, false );
	
	//Ip2_i = (mat *) calloc( sizeof(mat), N);
	Ip2_i = new mat[N];
	for(i=0; i<N; i++)  Ip2_i[i].set_size( S, M, false );	

        /* Turbo decoding */        
        intlvm_3d(Ip2, Ip2_i, ttcm->interleaver->length, ttcm->interleaver->table);
        
        for(i=0; i<ttcm->iterations; i++){
            /* 1st decoder */
            log_mapdec(N, M, S, Ps, Ns, Lb, Apr, Ip1, Apo, OPr);
            
            for (k=0; k<N; k++) for(m=0; m<M; m++){
	      if( Apr(k,m) > MINF ) Apr(k,m) = Apo(k,m) - Apr(k,m);
	      //Apr(k,m) = Apo(k,m) - Apr(k,m);
	    }
	    intlvd_2d(Apr, Apr_i, ttcm->interleaver->length, ttcm->interleaver->table);
            
            /* 2nd decoder */
            log_mapdec(N, M, S, Ps, Ns, Lb, Apr_i, Ip2_i, Apo_i, OPr2_i); 
            
            if(i==ttcm->iterations-1){
	      // the true extrinsic value has to be calculated here : based on the 2nd decoder's apriori 
	      //for(k=0; k<N; k++) for(m=0; m<M; m++){
	      //if( Apr_i(k,m) > MINF ) Apr_i(k,m) = Apo_i(k,m) - Apr_i(k,m);
		//Apr_i(k,m) = Apo_i(k,m) - Apr_i(k,m);
	      //}
	      //de_intlvd_2d(Apr_i, map->Extr, ttcm->interleaver->length, ttcm->interleaver->table);
	      break;
            }
	    
            for(k=0; k<N; k++) for(m=0; m<M; m++){
	      if( Apr_i(k,m) > MINF ) Apr_i(k,m) = Apo_i(k,m) - Apr_i(k,m);
	      //Apr_i(k,m) = Apo_i(k,m) - Apr_i(k,m);
	    }
            de_intlvd_2d(Apr_i, Apr, ttcm->interleaver->length, ttcm->interleaver->table);
        }
        
        de_intlvd_2d(Apo_i, Apo, ttcm->interleaver->length, ttcm->interleaver->table);
	/* Apo=real Apo, not extrinsic */
        
        de_intlvd_2d(OPr2_i, OPr2, ttcm->interleaver->length, ttcm->interleaver->table);
        
        for(i=0;i<N;i++)
          if(i%2!=0)
            for(m=0;m<2*M;m++)
              OPr(i,m) = OPr2(i,m);
	
	// free memory
	//for(i=0;i<N;i++) Ip2_i[i].~mat();  free(Ip2_i);
	delete [] Ip2_i;
	//Apr_i.~mat(); 
	//Apo_i.~mat();
	//OPr2_i.~mat();
}

void CM::SISO_dec_log(int N, int lD, int D, int S, imat Ps, imat Ns, imat Lb, mat *Apr_coded_bit,
		      mat *&Apo_coded_bit, mat *&Apo_data_bit, mat &Apo_dataword, int Terminated, int frame_index)
{
        int i, k, m, b, j, c;
        mat alpha, beta, Apr_codeword;  
        double max, Other_Apr;
	
        alpha = mat(N+1, S);
        beta  = mat(N+1, S);
        Apr_codeword = mat(N, 2*D);
	
        /* compute and normalise Apr_codeword */
        BitPr_to_Pr_log(N, lD+1, Apr_codeword, Apr_coded_bit, frame_index);
	
        /* compute and normalise alpha */
        for(i=0; i<S; i++) alpha(0,i)=MINF;
        alpha(0,0)=0.;
        for(k=1; k<=N; k++) 
        {   max = MINF;
            for(i=0; i<S; i++)
            {
                  alpha(k,i)=MINF;
                  for( c=0; c<2*D; c++){
                      if(Ps(i,c) != NO_LINK)
                          alpha(k,i) = jacolog( alpha(k,i), alpha(k-1,Ps(i,c)) + Apr_codeword(k-1,c));
                  }
                  if(max < alpha(k,i)) max = alpha(k,i);
            }
            for(i=0; i<S; i++) alpha(k,i) -= max;
        }

        /* compute and normalise beta */
        for(i=0; i<S; i++) beta(N,i)=0.;
        if(Terminated==ON) for(i=1; i<S; i++) beta(N,i)=MINF;

        for(k=N-1; k>=0; k--) 
        {   max = MINF;
            for(i=0; i<S; i++)
            { 
                 beta(k,i)=MINF;
                 for( m=0; m<D; m++)
                     beta(k,i) = jacolog( beta(k,i),
					  beta(k+1,Ns(i,m)) + Apr_codeword(k, Lb(i,m) ) );
                                             
                 if(max < beta(k,i)) max = beta(k,i);
            }
            for(i=0; i<S; i++) beta(k,i) -= max;
        }

        /* compute and normalise Apo_dataword & Apo_data_bit */
        for(k=0; k<N; k++) 
        {
            max = MINF; for(m=0;m<D;m++) Apo_dataword(k,m) = MINF;
            for(i=0; i<S; i++){
                for(c=0; c<2*D; c++){
                        
                    if( Ps(i,c)!=NO_LINK ){
                        for(j=0;j<D;j++){ if(c==Lb( Ps(i,c) ,j)) { m=j;break;} else m=NO_LINK; }
			
                        if(m==NO_LINK) s_error("Check CC table!");

                        Apo_dataword(k,m) =
                            jacolog(Apo_dataword(k,m),
				    alpha(k,Ps(i,c)) + beta(k+1,i) + Apr_codeword(k,c));

                        if(max < Apo_dataword(k,m)) max = Apo_dataword(k,m);
                    }
                }
            }
            for(m=0; m<D; m++)  Apo_dataword(k,m) -= max;
        }
        Pr_to_BitPr_log(N, lD, Apo_dataword, Apo_data_bit, frame_index);
	
        /* compute and normalise extrinsic Apo_coded_bit */
        for(k=0; k<N; k++){   
            for(b=0; b<=lD; b++){   
                Apo_coded_bit[b](k + frame_index*N,0) = Apo_coded_bit[b](k + frame_index*N,1) = MINF;
                for(c=0; c<2*D; c++){   
                    for(i=0; i<S; i++){
                        
                        if(Ps(i,c) != NO_LINK){
                            for(j=0, Other_Apr=0; j<=lD; j++) if(j!=b)
                                Other_Apr += Apr_coded_bit[j](k + frame_index*N, (c>>j)&1 );  
                            
                            /* when use 'Other_Apr' below, 'Apo_coded_bit' is extrinsic not real Apo */
                            Apo_coded_bit[b](k + frame_index*N, (c>>b)&1 ) =
                                jacolog(Apo_coded_bit[b](k + frame_index*N, (c>>b)&1 ),
					alpha(k,Ps(i,c)) + beta(k+1,i) + Other_Apr);
                        }
                        
                    } 
                } 
                //if(Apo_coded_bit[b](k + frame_index*N,0) < Apo_coded_bit[b](k + frame_index*N,1)) 
                //     max=Apo_coded_bit[b](k + frame_index*N,1);
                //else max=Apo_coded_bit[b](k + frame_index*N,0);
                //for(m=0; m<2; m++) Apo_coded_bit[b](k + frame_index*N,m) -= max;
            }
        }

        /* done */
        //alpha.~mat();
        //beta.~mat();
        //Apr_codeword.~mat();

  return;
}

void CM::SISO_dec_log(int N, int lD, int D, int S, imat Ps, imat Ns, imat Lb, mat Apr_codeword, mat Apr_dataword,
		      mat &Apo_codeword, mat &Apo_dataword, int Terminated, int frame_index)
{
        int i, k, m, b, j, c;
        mat alpha, beta;
        double max, codeword_max;
	
        alpha = mat(N+1, S);
        beta  = mat(N+1, S);
	
        /* compute and normalise alpha */
        for(i=0; i<S; i++) alpha(0,i)=MINF;
        alpha(0,0)=0.;
        for(k=1; k<=N; k++) 
        {   max = MINF;
            for(i=0; i<S; i++)
            {
                  alpha(k,i)=MINF;
                  for( c=0; c<2*D; c++){
		      if(Ps(i,c) != NO_LINK){

			  for(j=0;j<D;j++){ if(c==Lb( Ps(i,c) ,j)) { m=j;break;} else m=NO_LINK; }			  
			  if(m==NO_LINK) s_error("Check CC table!");
			  
                          alpha(k,i) = jacolog( alpha(k,i), 
						alpha(k-1,Ps(i,c)) + Apr_codeword(k-1,c) + Apr_dataword(k-1,m));
						//alpha(k-1,Ps(i,c)) + Apr_codeword(k-1,c) );
		      }
                  }
                  if(max < alpha(k,i)) max = alpha(k,i);
            }
            for(i=0; i<S; i++) alpha(k,i) -= max;
        }

        /* compute and normalise beta */
        for(i=0; i<S; i++) beta(N,i)=0.;
        if(Terminated==ON) for(i=1; i<S; i++) beta(N,i)=MINF;

        for(k=N-1; k>=0; k--) 
        {   max = MINF;
            for(i=0; i<S; i++)
            { 
                 beta(k,i)=MINF;
                 for( m=0; m<D; m++)
                     beta(k,i) = jacolog( beta(k,i),
					  beta(k+1,Ns(i,m)) + Apr_codeword(k, Lb(i,m) ) +  Apr_dataword(k,m) );
					  //beta(k+1,Ns(i,m)) + Apr_codeword(k, Lb(i,m) ) );
		 
                 if(max < beta(k,i)) max = beta(k,i);
            }
            for(i=0; i<S; i++) beta(k,i) -= max;
        }

        /* compute and normalise Apo_dataword & Apo_codeword */
        for(k=0; k<N; k++) 
        {
            max = MINF; for(m=0;m<D;m++) Apo_dataword(k,m) = MINF;
	    codeword_max = MINF; for(c=0;c<2*D;c++) Apo_codeword(k,c) = MINF;
	    
            for(i=0; i<S; i++){
                for(c=0; c<2*D; c++){
                        
                    if( Ps(i,c)!=NO_LINK ){

                        for(j=0;j<D;j++){ if(c==Lb( Ps(i,c) ,j)) { m=j;break;} else m=NO_LINK; }			
                        if(m==NO_LINK) s_error("Check CC table!");

                        Apo_dataword(k,m) =
                            jacolog(Apo_dataword(k,m),
				    alpha(k,Ps(i,c)) + beta(k+1,i) + Apr_codeword(k,c) + Apr_dataword(k,m));			
				    //alpha(k,Ps(i,c)) + beta(k+1,i) + Apr_codeword(k,c) );			
                        if(max < Apo_dataword(k,m)) max = Apo_dataword(k,m);
			
			Apo_codeword(k,c) =
			    jacolog(Apo_codeword(k,c),
				    alpha(k,Ps(i,c)) + beta(k+1,i) + Apr_codeword(k,c) + Apr_dataword(k,m));
				    //alpha(k,Ps(i,c)) + beta(k+1,i) + Apr_codeword(k,c) );
			if(codeword_max < Apo_codeword(k,c)) codeword_max = Apo_codeword(k,c);
			
                    }
                }
            }
            for(m=0; m<D; m++)  Apo_dataword(k,m) -= max;
	    for(c=0;c<2*D;c++)  Apo_codeword(k,c) -= codeword_max;
        }

        /* done */
        //alpha.~mat();
        //beta.~mat();

  return;
}

void CM::BICM_ID_dec_log(int N, int lD, int D, int S, imat Ps, imat Ns, imat Lb, mat *Apr_coded_bit, mat *Apr_data_bit,
			 mat *&Apo_coded_bit, mat *&Apo_data_bit, mat &Apo_dataword, int Terminated)
{
    int i, j, b, it, lC, nrof_codeword, nrof_dataword;
    mat *Extr_coded_bit;
    mat *Apo_coded_bit_i;
    mat Apr_codeword, Apr_dataword, Apo_codeword;
    
    lC=bicm->n;
    nrof_codeword = 1<<lC;
    nrof_dataword = 1<<lD;
    
    //printf("%d %d", nrof_codeword, nrof_dataword); getchar();
    
    //Extr_coded_bit = (mat *)calloc(sizeof(mat), lC);
    Extr_coded_bit = new mat[lC];
    for(i=0;i<lC;i++) Extr_coded_bit[i] = mat(N, 2); 
    
    //Apo_coded_bit_i = (mat *)calloc(sizeof(mat), lC);
    Apo_coded_bit_i = new mat[lC];
    for(i=0;i<lC;i++) Apo_coded_bit_i[i] = mat(N, 2); 
    
    Apr_codeword = mat(N, nrof_codeword);
    Apo_codeword = mat(N, nrof_codeword);
    Apr_dataword = mat(N, nrof_dataword);

    /* execute iterative decoding */
    for ( it = 0; it < bicmid->iterations; it++)
    {
        /* call SISO decoder to get extrinsic Apo */
        //SISO_dec_log(N, lD, D, S, Ps, Ns, Lb, 
                     //Apr_coded_bit, Apo_coded_bit, Apo_data_bit, 
		     //Apo_dataword, Terminated, 0);
      
        BitPr_to_Pr_log(N, lC, Apr_codeword, Apr_coded_bit, 0);
        BitPr_to_Pr_log(N, lD, Apr_dataword, Apr_data_bit,  0);

        SISO_dec_log(N, lD, D, S, Ps, Ns, Lb, 
                     Apr_codeword, Apr_dataword,
		     Apo_codeword, Apo_dataword, Terminated, 0);
	
	Pr_to_BitPr_log(N, lC, Apo_codeword, Apo_coded_bit,0);
        Pr_to_BitPr_log(N, lD, Apo_dataword, Apo_data_bit, 0);
      
	if(it == bicmid->iterations-1 ){	  
	  for(i=0;i<lC;i++) // interleave the APOSTERIORI coded bits for feeding back to Equalizer
	    bit_intlvm_3d( Apo_coded_bit, Apo_coded_bit_i, bicm->bit_interleaver[i]->length, 
			   2, bicm->bit_interleaver[i]->table, i );
	  for(b=0;b<lC;b++) siso->Apo_coded_bit[b] = Apo_coded_bit_i[b];

	  BitPr_to_Pr_log(N, lC, Apo_codeword, Apo_coded_bit,0);
	  OPr =  Apo_codeword;
	  break; // save time 
        }
        
	//compute extrinsic of coded bits 
 	for(b=0;b<lC;b++) for(i=0;i<N;i++) for(j=0;j<2;j++) Apo_coded_bit[b](i,j) -= Apr_coded_bit[b](i,j);
 	//for(b=0;b<lC;b++) for(i=0;i<N;i++) for(j=0;j<2;j++) if(Apr_coded_bit[b](i,j)>MINF) Apo_coded_bit[b](i,j) -= Apr_coded_bit[b](i,j);
	//for(b=0;b<lD;b++) for(i=0;i<N;i++) for(j=0;j<2;j++) if(Apr_data_bit[b](i,j)>MINF) Apo_data_bit[b](i,j) -= Apr_data_bit[b](i,j);
	
        /* interleave the Extrinsic_coded_bit */
        for(i=0;i<lC;i++)
	    bit_intlvm_3d( Apo_coded_bit, Apo_coded_bit_i, bicm->bit_interleaver[i]->length, 
			   2, bicm->bit_interleaver[i]->table, i );			   
        
        /* compute extrinsic info of demodulator without going into demodulator */
        extrinsic_demod_log(N, lD+1, 2*D, Apo_coded_bit_i, Pr,
                            Extr_coded_bit, 0); /*soft decision*/
        
        /* De-interleave the extrinsic Apo from modulator */
	for(i=0; i<lC; i++)
	    bit_de_intlvm_3d( Extr_coded_bit, Apr_coded_bit, bicm->bit_interleaver[i]->length, 
			      2, bicm->bit_interleaver[i]->table, i );     
    }/*end for(it=0; it<IT; it++)*/

    //for(i=0;i<lC;i++) Extr_coded_bit[i].~mat(); free(Extr_coded_bit);
    delete [] Extr_coded_bit;
    //for(i=0;i<bicm->n;i++) Apo_coded_bit_i[i].~mat(); free(Apo_coded_bit_i);
    delete [] Apo_coded_bit_i;

    //Apr_codeword.~mat();
    //Apo_codeword.~mat();
    //Apr_dataword.~mat();
}

void CM::extrinsic_demod_log(int N, int lC, int C, mat *Apo_coded_bit, mat Pr, mat *&Extr_coded_bit, int frame_index)
{
    int k, m, i, j;
    double max, Other_Apr;
  
    for(k=0; k<N; k++){ 
        for(i=0; i<lC; i++){
	    // initiliase 
	    Extr_coded_bit[i](k + frame_index*N,0) =  Extr_coded_bit[i](k + frame_index*N,1) = MINF;
            for(m=0; m<C; m++){
        
                for(j=0, Other_Apr=0; j<lC; j++) if(j!=i)
                    Other_Apr += Apo_coded_bit[j](k + frame_index*N, (m>>j)&1 );
        
                Extr_coded_bit[i](k + frame_index*N, (m>>i)&1 ) = 
		  jacolog( Extr_coded_bit[i](k + frame_index*N, (m>>i)&1 ), Pr(k,m) + Other_Apr);
            
            }
            if(Extr_coded_bit[i](k + frame_index*N,0) <  Extr_coded_bit[i](k + frame_index*N,1)) 
              max = Extr_coded_bit[i](k + frame_index*N,1);
            else max = Extr_coded_bit[i](k + frame_index*N,0);
            
            for(m=0; m<2; m++) Extr_coded_bit[i](k + frame_index*N,m) -= max;
        }
    }
}

void CM::BICM_ID_dec_log_pun(int N, int lD, int D, int S, imat Ps, imat Ns, imat Lb, mat *Apr_coded_bit, mat *Apr_data_bit,
			     mat *&Apo_coded_bit, mat *&Apo_data_bit, mat &Apo_dataword, int Terminated)
{
  /* 091003 mike: edited so that the following can be used. 
     SISO_dec_log(bicm->k*N, p_lD, p_D, p_S, Ps, Ns, Lb, 
       Apr_codeword, Apr_dataword,
       Apo_codeword, Apo_dataword_b, Terminated, 0);
     
     Apr_data_bit is also added to BICM_ID_dec_log_pun for joint source and channel coding
  */
    int i, j, it, lC, b;
    int p_lD, p_lC, p_D, p_S;
    mat *Extr_coded_bit, *Apo_coded_bit_i;
    mat *p_Apr_coded_bit, *p_Apo_coded_bit, *p_Apo_data_bit, *p_Apr_data_bit, p_Apo_dataword;
    
    lC=bicm->n;  p_lD=1;  p_lC=2;  p_D=1<<p_lD;  p_S=S;
    
    //Extr_coded_bit = (mat *)calloc(sizeof(mat), lC);
    Extr_coded_bit = new mat[lC];
    for(i=0;i<lC;i++) Extr_coded_bit[i] = mat(N, 2); 
    //Apo_coded_bit_i = (mat *)calloc(sizeof(mat), lC);
    Apo_coded_bit_i = new mat[lC];
    for(i=0;i<lC;i++) Apo_coded_bit_i[i] = mat(N, 2); 
    
    //p_Apr_coded_bit = (mat *)calloc(sizeof(mat), p_lC); 
    p_Apr_coded_bit = new mat[p_lC];
    for(i=0;i<p_lC;i++) p_Apr_coded_bit[i] = mat(bicm->k*N, 2);    
    //p_Apo_coded_bit = (mat *)calloc(sizeof(mat), p_lC); 
    p_Apo_coded_bit = new mat[p_lC];
    for(i=0;i<p_lC;i++) p_Apo_coded_bit[i] = mat(bicm->k*N, 2);        
    //p_Apo_data_bit = (mat *)calloc(sizeof(mat), p_lD); 
    p_Apo_data_bit = new mat[p_lD];
    for(i=0;i<p_lD;i++) p_Apo_data_bit[i] = mat(bicm->k*N, 2);
    p_Apr_data_bit = new mat[p_lD];
    for(i=0;i<p_lD;i++) p_Apr_data_bit[i] = mat(bicm->k*N, 2);
    p_Apo_dataword  = mat(bicm->k*N, p_D);
    
    //-----added on 091003 mike
    int nrof_codeword = 1<<p_lC;
    int nrof_dataword = 1<<p_lD;
    mat Apr_codeword, Apr_dataword, Apo_codeword, Apo_dataword_b;
    Apr_codeword = mat(bicm->k*N, nrof_codeword);
    Apo_codeword = mat(bicm->k*N, nrof_codeword);
    Apr_dataword = mat(bicm->k*N, nrof_dataword);
    Apo_dataword_b = mat(bicm->k*N, nrof_dataword);
    //-end-added on 091003 mike
    
    /* execute iterative decoding */
    for ( it = 0; it < bicmid->iterations; it++)
    {
        /*-------------------------------*/
        get_p_Apr_data_bit(p_Apr_data_bit, Apr_data_bit, N, bicm->k, p_lD);
        {
            depuncture(Apr_coded_bit, p_Apr_coded_bit, N, bicm->k, bicm->n, bicm->puncture, p_lC, 0);  
	    /* call SISO decoder to get extrinsic Apo */
            //SISO_dec_log(bicm->k*N, p_lD, p_D, p_S, Ps, Ns, Lb, 
	    //         p_Apr_coded_bit, p_Apo_coded_bit, p_Apo_data_bit, 
	    //         p_Apo_dataword, Terminated, 0);
            
	    //-----added on 091003 mike
	    BitPr_to_Pr_log(bicm->k*N, p_lD, Apr_dataword, p_Apr_data_bit,  0);	    
	    BitPr_to_Pr_log(bicm->k*N, p_lC, Apr_codeword, p_Apr_coded_bit, 0);
	    
	    SISO_dec_log(bicm->k*N, p_lD, p_D, p_S, Ps, Ns, Lb, 
			 Apr_codeword, Apr_dataword,
			 Apo_codeword, Apo_dataword_b, Terminated, 0);
	    
	    Pr_to_BitPr_log(bicm->k*N, p_lD, Apo_dataword_b, p_Apo_data_bit, 0);
	    Pr_to_BitPr_log(bicm->k*N, p_lC, Apo_codeword,   p_Apo_coded_bit,0);
	    //-END-added on 091003 mike
	    
            puncture(Apo_coded_bit, p_Apo_coded_bit, N, bicm->k, bicm->n, bicm->puncture, p_lC, 0);     
        }
        get_Apo_dataword(Apo_dataword, Apo_data_bit, p_Apo_data_bit, N, bicm->k, p_lD);        
        /*-------------------------------*/

	if(it == bicmid->iterations - 1 ){	  
	  for(i=0;i<lC;i++) // interleave the APOSTERIORI coded bits for feeding back to Equalizer
	    bit_intlvm_3d( Apo_coded_bit, Apo_coded_bit_i, bicm->bit_interleaver[i]->length, 
			   2, bicm->bit_interleaver[i]->table, i );
	  for(b=0;b<lC;b++) siso->Apo_coded_bit[b] = Apo_coded_bit_i[b];
	  
	  break; /* save time */
        }
        
	//compute extrinsic of coded bits
 	for(b=0;b<lC;b++) for(i=0;i<N;i++) for(j=0;j<2;j++) Apo_coded_bit[b](i,j) -= Apr_coded_bit[b](i,j);
 	

        /* interleave the Extrinsic_coded_bit*/
        for(i=0;i<lC;i++)
	    bit_intlvm_3d( Apo_coded_bit, Apo_coded_bit_i, bicm->bit_interleaver[i]->length, 
			   2, bicm->bit_interleaver[i]->table, i );	
        
        /* compute extrinsic info of demodulator without going into demodulator */
        extrinsic_demod_log(N, lD+1, 2*D, Apo_coded_bit_i, Pr, 
                            Extr_coded_bit, 0); /*soft decision*/
        
        /* De-interleave the extrinsic Apo from modulator */
        for(i=0; i<lC; i++)
	    bit_de_intlvm_3d( Extr_coded_bit, Apr_coded_bit, bicm->bit_interleaver[i]->length, 
			      2, bicm->bit_interleaver[i]->table, i );  
    }/*end for(it=0; it<IT; it++)*/
    
    
    /*-----------free memory-----------*/
    //for(i=0;i<lC;i++) Extr_coded_bit[i].~mat(); free(Extr_coded_bit);
    delete [] Extr_coded_bit;
    //for(i=0;i<bicm->n;i++) Apo_coded_bit_i[i].~mat(); free(Apo_coded_bit_i);
    delete [] Apo_coded_bit_i;

    //for(i=0;i<p_lC;i++) p_Apr_coded_bit[i].~mat(); free(p_Apr_coded_bit);
    delete [] p_Apr_coded_bit;
    //for(i=0;i<p_lC;i++) p_Apo_coded_bit[i].~mat(); free(p_Apo_coded_bit);
    delete [] p_Apo_coded_bit;
    //for(i=0;i<p_lD;i++) p_Apo_data_bit[i].~mat(); free(p_Apo_data_bit);
    delete [] p_Apo_data_bit;
    delete [] p_Apr_data_bit;
    //p_Apo_dataword.~mat();
}

void CM::depuncture(mat *Apr_coded_bit, mat *&p_Apr_coded_bit, int N, int dataword_length, int codeword_length,
		    ivec puncture, int p_lC, int frame_index)
{
    int i, j, k, m;
    
    ivec p_pattern;
    mat Apr_temp;
    mat p_Apr_temp;
    
    p_pattern.set_size(p_lC*dataword_length, false);
    Apr_temp   = mat(codeword_length*N, 2);
    p_Apr_temp = mat(p_lC*dataword_length*N, 2);
    
    /*copy into 1D binary array*/
    for(k=m=0; k<dataword_length; k++, m+=p_lC)
        for(i=0; i<p_lC; i++) /* p_lC=2 for 1/2-rate */
            p_pattern[m+i] = (puncture[k]>>i) & 1;

    //for(i=0; i<dataword_length*p_lC; i++) printf("%d ", p_pattern[i]); getchar();

    /*copy into 2D array*/
    for(k=m=0; k<N; k++, m+=codeword_length)
        for(i=0; i<codeword_length; i++)
            for(j=0; j<2; j++)
                Apr_temp(m+i,j) = Apr_coded_bit[i](k + frame_index*N,j);
    
    //for(i=0; i<N; i++) for(j=0; j<codeword_length; j++) printf("%.2f ", Apr_coded_bit[j](i + frame_index*N,0)); getchar();
    //for(i=0; i<codeword_length*N; i++) printf("%.2f ", Apr_temp(i,0)); getchar();
    
    /*add punctured bits*/
    for(k=m=0; k<N*codeword_length; )
        for(i=0; i<p_lC*dataword_length; i++)
            if(p_pattern[i]==1){  
              for(j=0; j<2; j++) p_Apr_temp(m,j) = Apr_temp(k,j);
              m++; k++;
            } 
            else{
              for(j=0; j<2; j++) p_Apr_temp(m,j) = -log(p_lC);
              m++;
            }

    //for(i=0; i<p_lC*dataword_length*N; i++) printf("%.2f ", p_Apr_temp(i,0)); getchar();
    
    /*put into output*/
    for(k=m=0; k<N*dataword_length; k++, m+=p_lC)
        for(i=0; i<p_lC; i++)
            for(j=0; j<2; j++)
                p_Apr_coded_bit[i](k + frame_index*N*dataword_length,j) = p_Apr_temp(m+i,j);

    //for(k=m=0; k<N*dataword_length; k++) for(i=0; i<p_lC; i++) printf("%d(%.2f)  ", m++, p_Apr_coded_bit[i](k + frame_index*N*dataword_length,0)); getchar();

    //p_pattern.~ivec();
    //Apr_temp.~mat();
    //p_Apr_temp.~mat();
}

void CM::puncture(mat *&Apo_coded_bit, mat *p_Apo_coded_bit, int N, int dataword_length, int codeword_length,
		  ivec puncture, int p_lC, int frame_index)
{
    int k, i, m, p;
    mat p_Apo_temp;

    p_Apo_temp = mat(N*codeword_length, 2);

    for(k=i=0; k<dataword_length*N; )
        for(p=0; p<dataword_length; p++, k++)
            for(m=0; m<p_lC; m++){
                if( (puncture[p]>>m) & 1 ){
                    p_Apo_temp(i,0) = p_Apo_coded_bit[m](k + frame_index*N*dataword_length,0);
                    p_Apo_temp(i,1) = p_Apo_coded_bit[m](k + frame_index*N*dataword_length,1);
                    i++;
                }
            }    

    //for(k=0; k<N*codeword_length; k++) printf("%.2f ", p_Apo_temp(k,0)); getchar();

    for(k=m=0; k<N; k++)
        for(i=0; i<codeword_length; i++){
            Apo_coded_bit[i](k + frame_index*N,0) = p_Apo_temp(m,0);
            Apo_coded_bit[i](k + frame_index*N,1) = p_Apo_temp(m,1);
            m++;
        }
    
    //for(k=0; k<N; k++){ for(i=0; i<codeword_length; i++) printf("%.2f ", Apo_coded_bit[i](k + frame_index*N,0)); printf("\n");}getchar();
    
    //p_Apo_temp.~mat();
}
void CM::get_p_Apr_data_bit(mat *&p_Apr_data_bit, mat *Apr_data_bit, int N, int dataword_length, int p_lD)
{
  int i, j, k, z, m;
  for(k=z=0; k<N; k++)
    for(m=0; m<dataword_length; m++, z++){
      for(i=0; i<p_lD; i++)
	for(j=0; j<2; j++) p_Apr_data_bit[i](z,j)= Apr_data_bit[m](k,j);
    }
}

void CM::get_Apo_dataword(mat &Apo_dataword, mat *&Apo_data_bit, mat *p_Apo_data_bit, int N, int dataword_length, int p_lD)
{
    /* 1/2-rate : p_lD = 1 */
    int k, z, i, j, m;
    
    for(k=z=0; k<N; k++)
        for(m=0; m<dataword_length; m++, z++){
            for(i=0; i<p_lD; i++)
                for(j=0; j<2; j++)  Apo_data_bit[m](k,j) = p_Apo_data_bit[i](z,j);
        }
    
    
    BitPr_to_Pr_log(N, dataword_length, Apo_dataword, Apo_data_bit, 0);

    //cout << Apo_dataword << endl; getchar();
}

void CM::reset_CM(int new_no_of_symbols)
{
  no_of_symbols = new_no_of_symbols;

  if( max_no_of_symbols < no_of_symbols ){
    printf("max_no_of_symbols=%d  no_of_symbols=%d\n",max_no_of_symbols,no_of_symbols);
    s_error("reset_CM : make sure max_no_of_symbols(fixed) >= no_of_symbols(variable)");
  }

  //cout<<no_of_symbols<<endl;
  
  if(Terminated==1){ 
    if(mode==TTCM) no_of_info_symbols = no_of_symbols-2*L;
    else           no_of_info_symbols = no_of_symbols-L;
  }
  else no_of_info_symbols = no_of_symbols;
  
  no_of_input_bits = no_of_symbols * k;
  no_of_info_bits = no_of_info_symbols * k;
  no_of_coded_bits = no_of_symbols * n;
  no_of_tail_bits = (no_of_symbols - no_of_info_symbols) * k;
    
  if(mode==TTCM){ 
    ttcm->interleaver->length = no_of_symbols;    
    ttcm->interleaver->table.set_length(ttcm->interleaver->length, false); 
    init_TTCM_interleaver();
  }
  
  free_channel_interleaver();
  init_channel_interleaver(no_of_symbols);  

  //cout<<no_of_info_symbols<<endl; getchar();
}

void CM::reset_CM_iteration(int new_iterations)
{
  if(mode==TTCM) ttcm->iterations   = new_iterations;
  else if(mode==BICMID) bicmid->iterations = new_iterations;  
}

void CM::initialise(){
  srand(0);
  
  if(system("mkdir interleaver_dir")==0){
    printf("interleaver_dir/ was created to store interleaver files\n\n");
  }
  else
    printf("interleaver files will be added into interleaver_dir/ \n\n");
  
  switch(mode)
    {
    case TCM:    
      allocate_memory_tcm();
      initTCM();
      initModulation_SP();
      break;
    case TTCM:
      allocate_memory_ttcm();
      initTCM();
      init_TTCM_interleaver();
      initModulation_SP();
      break;
    case BICM:    
      allocate_memory_bicm();
      
      modulation->mode=GRAY;
      bicmid->iterations = 1;
      
      if(bicm->k==5){ 
	bicm->puncture_code=ON;
	initPunctureCC();
      }
      else{
	bicm->puncture_code=OFF;
	initCC();
      }

      initModulation_GRAY();
      break;
    case BICMID:    
      allocate_memory_bicm();

      modulation->mode=SP;
      bicmid->iterations = iterations;
      
      if(bicm->k==5){ 
	bicm->puncture_code=ON;
	initPunctureCC();
      }
      else{
	bicm->puncture_code=OFF;
	initCC();
      }
      
      initModulation_SP();
      break;
    default:
      s_error("no such Coded Modulation");
      break;   
  }
  
  init_channel_interleaver(no_of_symbols);
}

void CM::allocate_memory_tcm()
{
    int i, M, S, N;
    
    //modulation = ( struct modulation_param *)malloc ( sizeof ( struct modulation_param ));
    modulation = new (struct modulation_param);
    modulation->mode=SP;
    modulation->type=modulation_type;
    modulation->bits_per_symbol=n; 

    modulation->number_of_levels=1<<n;
    modulation->SymbolMapping.set_size(modulation->number_of_levels, false);
    
    //tcm  = ( struct TCM_parameters *)malloc ( sizeof ( struct TCM_parameters ) );
    //map  = ( struct MAP_dec *)malloc ( sizeof ( struct MAP_dec ) );
    tcm  = new (struct TCM_parameters);
    map  = new (struct MAP_dec);

    tcm->k = k;
    tcm->L = L;

    //-------------------------------
    M = 1<<tcm->k;
    S = 1<<tcm->L;
        
    tcm->NoStates   = S;
    tcm->NoBranches = M;
    tcm->Lb = imat(S, M);
    tcm->Ns = imat(S, M);
    tcm->Ps = imat(S, M);
    
    N = no_of_symbols;
    
    Pr  = mat( N, 2*M );
    OPr = mat( N, 2*M );
    
    map->Apr = mat( N, M);
    map->Apo = mat( N, M);
    map->Extr.set_size( N, M, false);
    
    //map->Ip1 = (mat *)calloc( sizeof(mat), N);
    map->Ip1 = new mat[N]; 
    for(i=0; i<N; i++)  map->Ip1[i].set_size( S, M, false );
    
    tcm->GenPoly.set_size(tcm->k+1, false );
}

void CM::allocate_memory_ttcm()
{
    int i, M, S, N;
    
    //modulation = ( struct modulation_param *)malloc ( sizeof ( struct modulation_param ) );
    modulation = new ( struct modulation_param);
    modulation->mode=SP;
    modulation->type=modulation_type;
    modulation->bits_per_symbol=n;
    modulation->number_of_levels=1<<n;
    modulation->SymbolMapping.set_length(modulation->number_of_levels, false);
    
    //tcm  = ( struct TCM_parameters *)malloc ( sizeof ( struct TCM_parameters ) );
    //map  = ( struct MAP_dec *)malloc ( sizeof ( struct MAP_dec ) );
    tcm  = new ( struct TCM_parameters);
    map  = new ( struct MAP_dec);
    
    tcm->k = k;
    tcm->L = L;
    
    //ttcm = ( struct TTCM_parameters *) malloc ( sizeof ( struct TTCM_parameters ) );
    //ttcm->interleaver = ( struct cm_interleaver_control *) malloc ( sizeof ( struct cm_interleaver_control ) );
    ttcm = new ( struct TTCM_parameters );
    ttcm->interleaver = new ( struct cm_interleaver_control );
    
    ttcm->interleaver->length = no_of_symbols;
    ttcm->interleaver->table.set_length(ttcm->interleaver->length, false); 
    
    tcm->last.set_length(tcm->L, false); 
    
    ttcm->iterations = iterations;
    
    //-------------------------------
    M = 1<<tcm->k;
    S = 1<<tcm->L;
    
    tcm->NoStates   = S;
    tcm->NoBranches = M;
    tcm->Lb.set_size(S, M, false);
    tcm->Ns.set_size(S, M, false);
    tcm->Ps.set_size(S, M, false);
    
    N = no_of_symbols;

    Pr.set_size  ( N, 2*M, false );
    OPr.set_size ( N, 2*M, false );
    OPr2.set_size( N, 2*M, false );
    
    map->Apr.set_size( N, M, false);
    map->Apo.set_size( N, M, false);
    map->Extr.set_size( N, M, false);
    
    //map->Ip1 = (mat *)calloc( sizeof(mat), N);
    map->Ip1 = new mat[N];
    for(i=0; i<N; i++)  map->Ip1[i].set_size( S, M, false);
    //map->Ip2 = (mat *)calloc( sizeof(mat), N);
    map->Ip2 = new mat[N];
    for(i=0; i<N; i++)  map->Ip2[i].set_size( S, M, false);
    
    tcm->GenPoly.set_size(tcm->k+1, false );
}

void CM::allocate_memory_bicm()
{
    int i;
    int N, K, lD, lC, state, data;
    
    //modulation = ( struct modulation_param *)malloc ( sizeof ( struct modulation_param ) );
    modulation = new ( struct modulation_param );
    modulation->type=modulation_type;
    modulation->bits_per_symbol=n;
    modulation->number_of_levels=1<<n;
    modulation->SymbolMapping.set_length(modulation->number_of_levels, false);
    
    //cout << modulation->number_of_levels << endl;
    
    //bicm  = ( struct BICM_parameters *)malloc ( sizeof ( struct BICM_parameters ) );
    //siso  = ( struct SISO_dec *)malloc ( sizeof ( struct SISO_dec ) );
    //bicmid = ( struct BICMID_parameters *)malloc ( sizeof ( struct BICMID_parameters ) );
    bicm   = new ( struct BICM_parameters );
    siso   = new ( struct SISO_dec );
    bicmid = new ( struct BICMID_parameters );
    
    bicm->k = k;
    bicm->n = n;
    bicm->L = L;

    /*---------Initialise and memory allocation--------------------------*/
    N = no_of_symbols;

    K = bicm->L;    lD = bicm->k;    lC = bicm->n;
    state = 1<<K;               data = 1<<lD;
    
#ifdef debug_cm
    printf("\nN=%d, K=%d, lD=%d, lC=%d, state=%d, data=%d\n",N,K,lD,lC,state,data);
#endif
    
    bicm->NoStates   = state;
    bicm->NoBranches = data;

    bicm->GenPoly = imat(lD, lC);
    bicm->Lb      = imat(state, data);
    bicm->Ns      = imat(state, data);
    /* Note : Ps of BICM has more branches than TCM since codeword is used to label the branches */
    bicm->Ps      = imat(bicm->NoStates, 2*bicm->NoBranches);
    
    Pr            = mat( N, 2*data);
    OPr           = mat( N, 2*data);

    siso->Apo_dataword.set_size( N, data, false);
    
    //siso->Apo_data_bit  = (mat *)calloc(sizeof(mat), lD);
    siso->Apo_data_bit  = new mat[lD];
    for(i=0;i<lD;i++) siso->Apo_data_bit[i] = mat(N, 2);

    //siso->Apr_coded_bit = (mat *)calloc(sizeof(mat), lC);
    siso->Apr_coded_bit = new mat[lC];
    for(i=0;i<lC;i++) siso->Apr_coded_bit[i] = mat(N, 2); 
    //siso->Apo_coded_bit = (mat *)calloc(sizeof(mat), lC);
    siso->Apo_coded_bit = new mat[lC];
    for(i=0;i<lC;i++) siso->Apo_coded_bit[i] = mat(N, 2);   
    
    /* 1/2-rate punturing pattern for 64QAM */
    bicm->puncture.set_size(lD, false);
}

void CM::initTCM()
{
        int i, j, S, h, s, m, M, c, L;
	imat H;
        ivec D, K, cpoly;

        /*---------Initialise and memory allocation--------------------------*/        
        L = tcm->L; 

        //H = imat(tcm->k+1, tcm->L+1);
        H.set_size(tcm->k+1, tcm->L+1, false);

        D.set_size(tcm->L+1,false);
        K.set_size(tcm->k+1, false);
        cpoly.set_size(tcm->k+1, false);
        
        M = 1<<tcm->k;
        S = 1<<tcm->L;

        /*---------END_Initialise and memory allocation----------------------*/
        //printf("\nK = %d, L = %d\n",tcm->k, tcm->L);
        
        /* get generator polynomial depending on L and k */
        get_gen_poly();
        
        /* make a copy of poly */
        for(i=0; i<=tcm->k; i++) cpoly[i] = tcm->GenPoly[i];

        /* compute explicitely poli coeff from octal rep */ 
        h=0; 
        for (i=0; i <= L; i++)
        {
                /* extract last dec once every three times */
                h=h%3;
                if ( h == 0 )
                {
                        for( m=0; m<=tcm->k; m++ )
                        {       K[m] = cpoly[m]%10;     /* extract */
                            cpoly[m] = cpoly[m]/10;     /* shift */
                        }
                }
                h++;

                /* compute poli coeff */
                for( m=0; m<=tcm->k; m++ )
                {       
		    H(m,i) = K[m]%2;       /* compute */
                    K[m] = K[m]/2;              /* shift */
                }
        }
        /* a check */
        if ( H(0,0) != 1 ) s_error( "TCM: the feedback poly is inacceptable" );

        /* compute tables */
        for ( s = 0; s < S; s++ )
        {
            /* fill vector D (state) */
                h = s;
                for(i = 0; i<tcm->L; i++)
                {
                        D[i] = h%2;     
                        h = h / 2;
                }

                /* compute next state and output */
                for( m=0; m<M; m++ )
                {
                        /* fill vector K (input bits) */
                        h=m;
                        for(i=1; i<=tcm->k; i++)
                        {
                                K[i] = h%2;
                                h=h/2;
                        }

                        /* compute output bit (store in K[0] ) */
                        h = D[0];
                        for( i=1; i<tcm->k; i++)   h = ( h + K[i]*H(i,0) ) %2;
                        tcm->Lb(s,m) = 2*m + h;
                        K[0] = h;

                        /* compute new state */                 
                        c = 1; tcm->Ns(s,m) = 0;
                        for( j=0; j<tcm->L; j++)
                        {
                                /* bit from previous reg. */
                                if ( j < tcm->L-1 ) h=D[j+1];
                                else h=0;

                                /* input and feedback bits */
                                for( i=0; i<=tcm->k; i++)   h = ( h + K[i]*H(i,j+1) ) %2;
                        
                                /* add to state */
                                tcm->Ns(s,m) += h*c;
                                c = c*2;
                        }
                }
        }

        /* compute previous state table */
        for(i=0;i<S;i++)
          for(j=0;j<M;j++)
            tcm->Ps( tcm->Ns(i,j) ,j)=i;

        /*
        for(i=0;i<S;i++){
          for(j=0;j<M;j++){
            printf("s%d m%d  Ps%d Ns%d c%d", i, j, tcm->Ps(i,j),  
                  tcm->Ns(i,j), tcm->Lb(i,j));

            printf("\n");
          }
        }
	getchar();
        */
	
	// free mem 
	//H.~imat();
        //D.~ivec();
        //K.~ivec();
        //cpoly.~ivec();
}

void CM::get_gen_poly()
{
    /* generator polynimials */
    switch(tcm->k)
    {
        case 1:
            switch(L)
            {
                case 1:
                    tcm->GenPoly[0] = 1;  tcm->GenPoly[1] = 2;
                    break;            
                case 3:
                    tcm->GenPoly[0] = 13;  tcm->GenPoly[1] = 6;
                    break;
                case 4:
                    tcm->GenPoly[0] = 23;  tcm->GenPoly[1] = 6;
                    break;
                case 6:
                    tcm->GenPoly[0] = 117; tcm->GenPoly[1] = 26; 
                    break;
                case 7:
                    tcm->GenPoly[0] = 217; tcm->GenPoly[1] = 110;
                    break;
                case 8:
                    tcm->GenPoly[0] = 427; tcm->GenPoly[1] = 230;
                    break;
                case 9:
                    tcm->GenPoly[0] = 1017; tcm->GenPoly[1] = 120;
                    break;
                default:
                    s_error("no generator for such code yet for 4QAM");
                    break;                
            }
            break;
        case 2:
            switch(L)
            {
	        //case 1: //dummy
                //    tcm->GenPoly[0] = 1;  tcm->GenPoly[1] = 2;  tcm->GenPoly[2] = 0;
                //    break;
                case 3:
                    tcm->GenPoly[0] = 11;  tcm->GenPoly[1] = 2;  tcm->GenPoly[2] = 4;
                    break;
                case 4:
                    tcm->GenPoly[0] = 23;  tcm->GenPoly[1] = 2;  tcm->GenPoly[2] = 10;
                    break;
                case 6:
                    tcm->GenPoly[0] = 103; tcm->GenPoly[1] = 30; tcm->GenPoly[2] = 66;
                    break;
                case 7:
                    tcm->GenPoly[0] = 277; tcm->GenPoly[1] = 54; tcm->GenPoly[2] = 122;
                    break;
                case 8:
                    tcm->GenPoly[0] = 435; tcm->GenPoly[1] = 72; tcm->GenPoly[2] = 130;
                    break;
                default:
                    s_error("no generator for such code yet, for 8PSK");
                    break;                
            }
            break;
        case 3:
            switch(L)
            {
                case 3:
                    tcm->GenPoly[0] = 11;  tcm->GenPoly[1] = 2;  tcm->GenPoly[2] = 4;  tcm->GenPoly[3] = 10;
                    break;
                case 4:
                    tcm->GenPoly[0] = 23;  tcm->GenPoly[1] = 2;  tcm->GenPoly[2] = 4;  tcm->GenPoly[3] = 10;
                    break;
                case 6:
                    tcm->GenPoly[0] = 101; tcm->GenPoly[1] = 16; tcm->GenPoly[2] = 64; tcm->GenPoly[3] = 0;
                    break;
                case 7:
                    tcm->GenPoly[0] = 203; tcm->GenPoly[1] = 14; tcm->GenPoly[2] = 42; tcm->GenPoly[3] = 10; 
                    break;
                default:
                    s_error("no generator for such code yet, for 16QAM");
                    break;                
            }
            break;
        case 4:
            switch(L)
            {
	        case 3: // hand design based on Ungerbock's and Robertson's code
                    tcm->GenPoly[0] = 11; tcm->GenPoly[1] = 2;  tcm->GenPoly[2] = 4;
                    tcm->GenPoly[3] = 10;  tcm->GenPoly[4] = 0;  
                    break;
	        case 4: // hand design
		  //tcm->GenPoly[0] = 21;  tcm->GenPoly[1] = 2;  tcm->GenPoly[2] = 4;  
		  //tcm->GenPoly[3] = 10;  tcm->GenPoly[4] = 20;
                    tcm->GenPoly[0] = 37;  tcm->GenPoly[1] = 2;  tcm->GenPoly[2] = 4;  
		    tcm->GenPoly[3] = 10;  tcm->GenPoly[4] = 20; 
		    break;
	        case 5: 
                    tcm->GenPoly[0] = 41;  tcm->GenPoly[1] = 2;  tcm->GenPoly[2] = 4;  
		    tcm->GenPoly[3] = 10;  tcm->GenPoly[4] = 20; 
		    break;
                case 6:  
                    tcm->GenPoly[0] = 101; tcm->GenPoly[1] = 16;  tcm->GenPoly[2] = 64; 
                    tcm->GenPoly[3] = 0;   tcm->GenPoly[4] = 0; 
                    break;    
                default:
                    s_error("no generator for such code yet, for 32QAM");
                    break;
            }
            break;
        case 5:
            switch(L)
            {
                case 3:
                    tcm->GenPoly[0] = 11; tcm->GenPoly[1] = 2;  tcm->GenPoly[2] = 4;
                    tcm->GenPoly[3] = 0;  tcm->GenPoly[4] = 0;  tcm->GenPoly[5] = 0;
                    break;
                case 5:  /*this is derived from k=3,L=6 case above 16.3.00*/
                    tcm->GenPoly[0] = 41; tcm->GenPoly[1] = 2;  tcm->GenPoly[2] = 4; 
                    tcm->GenPoly[3] = 10;   tcm->GenPoly[4] = 20;   tcm->GenPoly[5] = 40;
                    break;    
                case 6:  /*this is derived from k=3,L=6 case above 16.3.00*/
                    tcm->GenPoly[0] = 101; tcm->GenPoly[1] = 16;  tcm->GenPoly[2] = 64; 
                    tcm->GenPoly[3] = 0;   tcm->GenPoly[4] = 0;   tcm->GenPoly[5] = 0;
                    break;    
                default:
                    s_error("no generator for such code yet, for 64QAM");
                    break;
            }
            break;
        case 7:
            switch(L)
            {
                case 3:
                    tcm->GenPoly[0] = 11; tcm->GenPoly[1] = 2;  tcm->GenPoly[2] = 4;
                    tcm->GenPoly[3] = 0;  tcm->GenPoly[4] = 0;  tcm->GenPoly[5] = 0;
                    tcm->GenPoly[6] = 0;  tcm->GenPoly[7] = 0;
                    break;
                default:
                    s_error("no generator for such code yet, for 128QAM");
                    break;                
            }
            break;
        default:
            s_error("no generator for such code yet");
            break;
    }
    
}


/* ======================================== BICM ===============================*/
/* Code tools for modular simulator. Author: Michael Ng */
/* lD = dataword length; lC = codeword length; M = code memory per input bit; N = block length; */
/* K = total code memory; gen = gen poly vector; Ns = next state table; Lb = codeword table; */
void CM::initCC()
{
  int i, s, m, k, test, M;
  int state;     /* number of state */
  int data;      /* number of data word */
  int d, c, sum;
  ivec lM;       /* actual code memory per data bit */
  imat cwb;     /* codeword bits of individual state */
  imat *cw;     /* codeword of individual state */
  ivec com_s;    /* state corresponding to a component */
  ivec com_d;    /* data corresponding to a component */

  int lD, lC, K;
  imat gen;  

  /*---------Initialise and memory allocation--------------------------*/
  K = bicm->L;    lD = bicm->k;    lC = lD + 1;

  gen = imat(lD, lC);
  
  lM.set_size(lD, false); 
  cwb= imat(lD, lC);

  com_s.set_size(lD);
  com_d.set_size(lD);

  //printf("K = %d, L = %d\n", bicm->k, bicm->L);	
      /* get generator polynomial depending on L and k */
  get_CC_poly();

      /* make a copy of poly */
  for(d=0; d<lD; d++) for(c=0; c<lC; c++) gen(d,c) = bicm->GenPoly(d,c);
  M = bicm->M;
  
  //cw = (imat *) calloc(sizeof(imat), lD);
  cw = new imat[lD];
  for(i=0; i<lD; i++) cw[i].set_size((int)pow(2,M), 2, false);

  /*---------END_Initialise and memory allocation----------------------*/

  
  /*get the equivalent of decimal generator, for shift function at a later state*/
  for(i=0;i<lD;i++) for(c=0;c<lC;c++)
    gen(i,c) = octal_to_decimal(gen(i,c));
  
  /*get the exact shift register state for each input bit and the exact generator polynomial*/
  for(i=0; i<lD; i++) for(m=0,test=0,lM[i]=M; m<M; m++){
    for(c=0; c<lC; c++)
      test += gen(i,c)&1; 
      
    if(test==0){ lM[i]--; for(c=0;c<lC;c++) gen(i,c)=gen(i,c)>>1; }
    else m=M;
  }

  for(m=0,sum=0; m<lD; m++) {
#ifdef debug_cm
    printf("\nlM[%d] = %d", m, lM[m]); 
#endif
    sum+=lM[m];
  } 
  if(K!=sum) s_error("(initCC) Conflict state number K with lM.");

#ifdef debug_cm
  printf("\ngen in decimal:");
  for(i=0;i<lD;i++) {printf("\ngen[%d] = ",i); for(c=0;c<lC;c++) printf("%d ",gen(i,c));}printf("\n");
#endif
  
  /* generate label table for individual section */
  for(i=0;i<lD;i++){
    state = (int)pow(2,lM[i]);
    for(d=0;d<2;d++){
      for(s=0;s<state;s++){
	
	for(c=0;c<lC;c++){
	  cwb(i,c) = (d&1)*((gen(i,c)>>lM[i])&1);

	  for(k=lM[i]-1; k>=0; k--){
	    cwb(i,c) += ((s>>k)&1)*((gen(i,c)>>k)&1);
	  }

	  if(cwb(i,c)%2) cwb(i,c)=1;  /*simple modulo two adder*/
	  else cwb(i,c)=0;

	  //printf("\nd=%d, s=%d,\tcwb(%d,%d)=%d",d,s,i,c,cwb(i,c));
	}

	cw[i](s,d) = cwb(i,0);
	for(c=1;c<lC;c++)
	  cw[i](s,d) += cwb(i,c)*(int)pow(2,c) ;
      }    
    }
  }

  /* print sub table */
  /*
  for(i=0;i<lD;i++){
    state = pow(2,lM[i]);
    printf("\n-------------------");
    for(d=0;d<2;d++){
      printf("\n");
      for(s=0;s<state;s++){
	printf("cw[%d](%d,%d)=%d ",i,s,d,cw[i](s,d));
      }
    }
  }printf("\n");
  */

  /* generate label table */
  state = (int) pow(2,K);   data = (int) pow(2,lD);
  for(s=0;s<state;s++){
    for(d=0;d<data;d++){

      for(i=0,sum=0,k=0;i<lD;i++){
	com_d[i] = (d>>i) & 1;

	if(i==0) com_s[i] = s & ( (int)pow(2,lM[i]) - 1 );
	else{
	  k += lM[i-1];
	  com_s[i] = ( s>>k ) & ( (int)pow(2,lM[i]) - 1 );
	}
	//printf("\nd=%d -> com_d[%d]=%d\t\ts=%d -> com_s[%d]=%d",d,i,com_d[i],s,i,com_s[i]);
	sum ^= cw[i]( com_s[i] , com_d[i] );
      }
      
      bicm->Lb(s,d) = sum;
      //printf("\nd=%d s=%d, bicm->Lb=%d",d,s,bicm->Lb(s,d));
    }
  }
  
  /* compute next state */
  for(s=0;s<state;s++){
    for(d=0;d<data;d++){
      
      for(i=0,sum=0,k=0;i<lD;i++){

	if(i==0) com_s[i] = s & ( (int)pow(2,lM[i]) - 1 );
	else{
	  k += lM[i-1];
	  com_s[i] = ( s>>k ) & ( (int)pow(2,lM[i]) - 1 );
	}

	com_s[i]>>= 1;
	com_s[i] ^= (int)pow(2,lM[i]-1)*( (d>>i)&1 );

	//printf("\nd=%d s=%d -> com_s[%d]=%d",d,s,i,com_s[i]);	
	sum ^= com_s[i]<<k;
      }
      
      bicm->Ns(s,d)=sum;
      //printf("\nd=%d s=%d, Ns=%d",d,s,bicm->Ns(s,d));
    }
  }

  
  /* compute previous state */
  for(i=0; i<bicm->NoStates; i++) for(d=0; d<2*bicm->NoBranches;d++)
      bicm->Ps(i,d)=NO_LINK;                        
  for(i=0; i<bicm->NoStates; i++) for(d=0; d<bicm->NoBranches;d++)
  {
      bicm->Ps( bicm->Ns(i,d) , bicm->Lb(i,d) )
          = i;
  }
  
  /*
  for(i=0; i<bicm->NoStates; i++){ 
      for(d=0; d<bicm->NoBranches;d++){
          printf(" %d",bicm->Ns(i,d));
      }
      printf("\n");
  }
  printf("\n\n"); 
  for(i=0; i<bicm->NoStates; i++){ 
      for(d=0; d<2*bicm->NoBranches;d++){
          printf(" %d",bicm->Ps( i , d ));
      }
      printf("\n");
  }
  getchar();
  */

  /* free memory */
  //lM.~ivec();
  //gen.~imat();
  //cwb.~imat();
  
  //for(i=0; i<lD; i++) cw[i].~imat(); free(cw);
  delete [] cw;
  //com_s.~ivec();
  //com_d.~ivec();
}


void CM::get_CC_poly()
{
    /* generator polynimials */
    switch(bicm->k)
    {
        case 1:
	    switch(bicm->L)
	    {
		case 3:
		    bicm->M = 3;
		    bicm->GenPoly(0,0) = 15;   bicm->GenPoly(0,1) = 17;
		    break;
		case 4:
		    bicm->M = 4;
		    bicm->GenPoly(0,0) = 23;   bicm->GenPoly(0,1) = 35;
		    break;
		case 5:
		    bicm->M=5;
		    bicm->GenPoly(0,0) = 53;   bicm->GenPoly(0,1) = 75;
		    break;
		case 6:
		    bicm->M = 6;
		    bicm->GenPoly(0,0) = 133;  bicm->GenPoly(0,1) = 171;
		    break;
		default:
		    s_error("no generator for such code yet, for BICM-4QAM");
		break;
	    }
            break;
        case 2:
            switch(bicm->L)
            {
                case 3:
                    bicm->M = 2;
                    bicm->GenPoly(0,0) = 4;  bicm->GenPoly(0,1) = 2;  bicm->GenPoly(0,2) = 6;
                    bicm->GenPoly(1,0) = 1;  bicm->GenPoly(1,1) = 4;  bicm->GenPoly(1,2) = 7;
                    break;
                case 4:
                    bicm->M = 2;
                    bicm->GenPoly(0,0) = 7;  bicm->GenPoly(0,1) = 1;  bicm->GenPoly(0,2) = 4;
                    bicm->GenPoly(1,0) = 2;  bicm->GenPoly(1,1) = 5;  bicm->GenPoly(1,2) = 7;
                    break;
                case 5:
                    bicm->M = 3;
                    bicm->GenPoly(0,0) = 14;  bicm->GenPoly(0,1) = 06;  bicm->GenPoly(0,2) = 16;
                    bicm->GenPoly(1,0) = 03;  bicm->GenPoly(1,1) = 10;  bicm->GenPoly(1,2) = 17;
                    break;
                case 6:
                    bicm->M = 3;
                    bicm->GenPoly(0,0) = 15;  bicm->GenPoly(0,1) = 06;  bicm->GenPoly(0,2) = 15;
                    bicm->GenPoly(1,0) = 06;  bicm->GenPoly(1,1) = 15;  bicm->GenPoly(1,2) = 17;
                    break;
                default:
                    s_error("no generator for such code yet, for BICM-8PSK");
                    break;                
            }
            break;
        case 3:
            switch(bicm->L)
            {
                case 3:
                    bicm->M = 2;
                    bicm->GenPoly(0,0) = 4;  bicm->GenPoly(0,1) = 4;  bicm->GenPoly(0,2) = 4;  bicm->GenPoly(0,3) = 4;
                    bicm->GenPoly(1,0) = 0;  bicm->GenPoly(1,1) = 6;  bicm->GenPoly(1,2) = 2;  bicm->GenPoly(1,3) = 4;
                    bicm->GenPoly(2,0) = 0;  bicm->GenPoly(2,1) = 2;  bicm->GenPoly(2,2) = 5;  bicm->GenPoly(2,3) = 5;
                    break;
		case 5:
		    bicm->M = 2;
		    bicm->GenPoly(0,0) = 6;  bicm->GenPoly(0,1) = 2;  bicm->GenPoly(0,2) = 2;  bicm->GenPoly(0,3) = 6;
		    bicm->GenPoly(1,0) = 1;  bicm->GenPoly(1,1) = 6;  bicm->GenPoly(1,2) = 0;  bicm->GenPoly(1,3) = 7;
		    bicm->GenPoly(2,0) = 0;  bicm->GenPoly(2,1) = 2;  bicm->GenPoly(2,2) = 5;  bicm->GenPoly(2,3) = 5;
		    break;
                case 6:
                    bicm->M = 2;
                    bicm->GenPoly(0,0) = 6;  bicm->GenPoly(0,1) = 1;  bicm->GenPoly(0,2) = 0;  bicm->GenPoly(0,3) = 7;
                    bicm->GenPoly(1,0) = 3;  bicm->GenPoly(1,1) = 4;  bicm->GenPoly(1,2) = 1;  bicm->GenPoly(1,3) = 6;
                    bicm->GenPoly(2,0) = 2;  bicm->GenPoly(2,1) = 3;  bicm->GenPoly(2,2) = 7;  bicm->GenPoly(2,3) = 4;
                    break;
                default:
                    s_error("no generator for such code yet, for BICM-16QAM");
                    break;                
            }
            break;
        case 5:
	    switch(bicm->L)
            {
	      case 3:
		  bicm->M = 3;
		  bicm->GenPoly(0,0) = 15;   bicm->GenPoly(0,1) = 17;

		  /* puncturing pattern */
		  bicm->puncture[0] = 1; bicm->puncture[1] = 2; bicm->puncture[2] = 2; bicm->puncture[3] = 3; bicm->puncture[4] = 2; 
		  break;
	      case 6:
		  bicm->M = 6;
		  bicm->GenPoly(0,0) = 133;   bicm->GenPoly(0,1) = 171;

		  /* puncturing pattern */
		  bicm->puncture[0] = 3; bicm->puncture[1] = 1; bicm->puncture[2] = 1; bicm->puncture[3] = 1; bicm->puncture[4] = 1; 
		  break;
	      default:
		  s_error("no generator for such code yet, for BICM-64QAM puntured code.");
		  break; 	
	    }
	    break;
        default:
            s_error("Covolutional code : no generator for such code yet");
            break;
    }
}


/*-----------------puncture code : 1/2-rate mother code ----------------------------------*/
void CM::initPunctureCC()
{
  int i, s, m, k, test, M, PuncBranches;
  int state;     /* number of state */
  int data;      /* number of data word */
  int d, c, sum;
  ivec lM;       /* actual code memory per data bit */
  imat cwb;     /* codeword bits of individual state */
  imat *cw;     /* codeword of individual state */
  ivec com_s;    /* state corresponding to a component */
  ivec com_d;    /* data corresponding to a component */

  int lD, lC, K;
  imat gen;
  
  /*---------Initialise and memory allocation--------------------------*/
  K = bicm->L;    
  lD = 1;    
  lC = 2; 
  PuncBranches=2;
  
  gen = imat(lD, lC);
  
  lM.set_size(lD, false); 
  cwb= imat(lD, lC);
  
  com_s.set_size(lD);
  com_d.set_size(lD);
  
#ifdef debug_cm
  printf("K = %d, L = %d\n",bicm->k, bicm->L);  
#endif
      /* get generator polynomial depending on L and k */
  get_CC_poly();

      /* make a copy of poly */
  for(d=0; d<lD; d++) for(c=0; c<lC; c++) gen(d,c) = bicm->GenPoly(d,c);
  M = bicm->M;
  
  //cw = (imat *)calloc(sizeof(imat), lD);
  cw = new imat[lD];
  for(i=0; i<lD; i++) cw[i] = imat((int)pow(2,M), 2);

  /*---------END_Initialise and memory allocation----------------------*/



  /*get the equivalent of decimal generator, for shift function at a later state*/
  for(i=0;i<lD;i++) for(c=0;c<lC;c++)
    gen(i,c) = octal_to_decimal(gen(i,c));
  
  /*get the exact shift register state for each input bit and the exact generator polynomial*/
  for(i=0; i<lD; i++) for(m=0,test=0,lM[i]=M; m<M; m++){
    for(c=0; c<lC; c++)
      test += gen(i,c)&1; 
      
    if(test==0){ lM[i]--; for(c=0;c<lC;c++) gen(i,c)=gen(i,c)>>1; }
    else m=M;
  }
  for(m=0,sum=0; m<lD; m++) {
#ifdef debug_cm
    printf("\nlM[%d] = %d", m, lM[m]); 
#endif
    sum+=lM[m];
  } 
  if(K!=sum) s_error("(initCC) Conflict state number K with lM.");

#ifdef debug_cm
  printf("\ngen in decimal:");
  for(i=0;i<lD;i++) {printf("\ngen[%d] = ",i); for(c=0;c<lC;c++) printf("%d ",gen(i,c));}printf("\n");
#endif

  /* generate label table for individual section */
  for(i=0;i<lD;i++){
    state = (int)pow(2,lM[i]);
    for(d=0;d<2;d++){
      for(s=0;s<state;s++){
        
        for(c=0;c<lC;c++){
          cwb(i,c) = (d&1)*((gen(i,c)>>lM[i])&1);

          for(k=lM[i]-1; k>=0; k--){
            cwb(i,c) += ((s>>k)&1)*((gen(i,c)>>k)&1);
          }

          if(cwb(i,c)%2) cwb(i,c)=1;  /*simple modulo two adder*/
          else cwb(i,c)=0;

          //printf("\nd=%d, s=%d,\tcwb(%d,%d)=%d",d,s,i,c,cwb(i,c));
        }

        cw[i](s,d) = cwb(i,0);
        for(c=1;c<lC;c++)
          cw[i](s,d) += cwb(i,c)*(int)pow(2,c) ;
      }    
    }
  }

  /* print sub table */
  /*
  for(i=0;i<lD;i++){
    state = pow(2,lM[i]);
    printf("\n-------------------");
    for(d=0;d<2;d++){
      printf("\n");
      for(s=0;s<state;s++){
        printf("cw[%d](%d,%d)=%d ",i,s,d,cw[i](s,d));
      }
    }
  }printf("\n");
  */

  /* generate label table */
  state = (int)pow(2,K);   data = (int)pow(2,lD);
  for(s=0;s<state;s++){
    for(d=0;d<data;d++){

      for(i=0,sum=0,k=0;i<lD;i++){
        com_d[i] = (d>>i) & 1;

        if(i==0) com_s[i] = s & ( (int)pow(2,lM[i]) - 1 );
        else{
          k += lM[i-1];
          com_s[i] = ( s>>k ) & ( (int)pow(2,lM[i]) - 1 );
        }
        //printf("\nd=%d -> com_d[%d]=%d\t\ts=%d -> com_s[%d]=%d",d,i,com_d[i],s,i,com_s[i]);
        sum ^= cw[i]( com_s[i] , com_d[i] );
      }
      
      bicm->Lb(s,d) = sum;
      //printf("\nd=%d s=%d, bicm->Lb=%d",d,s,bicm->Lb(s,d));
    }
  }
  
  /* compute next state */
  for(s=0;s<state;s++){
    for(d=0;d<data;d++){
      
      for(i=0,sum=0,k=0;i<lD;i++){

        if(i==0) com_s[i] = s & ( (int)pow(2,lM[i]) - 1 );
        else{
          k += lM[i-1];
          com_s[i] = ( s>>k ) & ( (int)pow(2,lM[i]) - 1 );
        }

        com_s[i]>>= 1;
        com_s[i] ^= (int)pow(2,lM[i]-1)*( (d>>i)&1 );

        //printf("\nd=%d s=%d -> com_s[%d]=%d",d,s,i,com_s[i]); 
        sum ^= com_s[i]<<k;
      }
      
      bicm->Ns(s,d)=sum;
      //printf("\nd=%d s=%d, Ns=%d",d,s,bicm->Ns(s,d));
    }
  }

  
  /* compute previous state */
  for(i=0; i<bicm->NoStates; i++) for(d=0; d<2*PuncBranches;d++)
      bicm->Ps(i,d)=NO_LINK;                        
  for(i=0; i<bicm->NoStates; i++) for(d=0; d<PuncBranches;d++)
  {
      bicm->Ps( bicm->Ns(i,d) , bicm->Lb(i,d) )
          = i;
  }

  /*
  for(i=0; i<bicm->NoStates; i++){ 
      for(d=0; d<PuncBranches;d++){
          printf(" %d",bicm->Ns(i,d));
      }
      printf("\n");
  }
  printf("\n\n"); 
  for(i=0; i<bicm->NoStates; i++){ 
      for(d=0; d<2*PuncBranches;d++){
          printf(" %d",bicm->Ps( i , d ));
      }
      printf("\n");
  }
  getchar();
  */

  /* free memory */
  //lM.~ivec();
  //gen.~imat();
  //cwb.~imat();
  //for(i=0; i<lD; i++) cw[i].~imat(); free(cw);
  delete [] cw;
  //com_s.~ivec();
  //com_d.~ivec();
}






/*--- modulation module --------------------------------------------*/

// non-square QAM
void CM::APSK_32_modulation()
{
  modulation->SymbolMapping[0] = complex<double>(-1.080123, -1.080123);  // 0
  modulation->SymbolMapping[1] = complex<double>(-0.771517, -0.771517);  // 2
  modulation->SymbolMapping[2] = complex<double>(-0.462910, -1.080123);  // 4
  modulation->SymbolMapping[3] = complex<double>(-0.154303, -0.771517);  // 6
  modulation->SymbolMapping[4] = complex<double>(-0.462910, -0.462910);  // 8
  modulation->SymbolMapping[5] = complex<double>(-0.154303, -0.154303);  // 10
  modulation->SymbolMapping[6] = complex<double>(-1.080123, -0.462910);  // 12
  modulation->SymbolMapping[7] = complex<double>(-0.771517, -0.154303);  // 14
  modulation->SymbolMapping[8] = complex<double>(0.154303, -1.080123);  // 16
  modulation->SymbolMapping[9] = complex<double>(0.462910, -0.771517);  // 18
  modulation->SymbolMapping[10] = complex<double>(0.771517, -1.080123);  // 20
  modulation->SymbolMapping[11] = complex<double>(1.080123, -0.771517);  // 22
  modulation->SymbolMapping[12] = complex<double>(0.771517, -0.462910);  // 24
  modulation->SymbolMapping[13] = complex<double>(1.080123, -0.154303);  // 26
  modulation->SymbolMapping[14] = complex<double>(0.154303, -0.462910);  // 28
  modulation->SymbolMapping[15] = complex<double>(0.462910, -0.154303);  // 30
  modulation->SymbolMapping[16] = complex<double>(0.154303, 0.154303);  // 32
  modulation->SymbolMapping[17] = complex<double>(0.462910, 0.462910);  // 34
  modulation->SymbolMapping[18] = complex<double>(0.771517, 0.154303);  // 36
  modulation->SymbolMapping[19] = complex<double>(1.080123, 0.462910);  // 38
  modulation->SymbolMapping[20] = complex<double>(0.771517, 0.771517);  // 40
  modulation->SymbolMapping[21] = complex<double>(1.080123, 1.080123);  // 42
  modulation->SymbolMapping[22] = complex<double>(0.154303, 0.771517);  // 44
  modulation->SymbolMapping[23] = complex<double>(0.462910, 1.080123);  // 46
  modulation->SymbolMapping[24] = complex<double>(-1.080123, 0.154303);  // 48
  modulation->SymbolMapping[25] = complex<double>(-0.771517, 0.462910);  // 50
  modulation->SymbolMapping[26] = complex<double>(-0.462910, 0.154303);  // 52
  modulation->SymbolMapping[27] = complex<double>(-0.154303, 0.462910);  // 54
  modulation->SymbolMapping[28] = complex<double>(-0.462910, 0.771517);  // 56
  modulation->SymbolMapping[29] = complex<double>(-0.154303, 1.080123);  // 58
  modulation->SymbolMapping[30] = complex<double>(-1.080123, 0.771517);  // 60
  modulation->SymbolMapping[31] = complex<double>(-0.771517, 1.080123);  // 62
}


void CM::initModulation_SP()
{
    int M; 

    M = modulation->number_of_levels;
  
    if (M<2 || M>64) s_error("Illegal number of constellation points");
    
    switch( modulation->type )
    {
        case CPSK:
        {
	    double pi;
            int j;
            
            pi=acos(-1.);
            for(j=0; j<M; j++){
		modulation->SymbolMapping[j] = 
		    //double_complex(cos((2.*pi*j)/M),sin((2.*pi*j)/M) );
		    complex<double>(cos((2.*pi*j)/M),sin((2.*pi*j)/M) );
	    }
            break;
        }
        case CQAM:
        {
            int i, j;
            imat lab, bit1, bit2; 
	    int s, N, lN, pos;
            double f;
            
                /* check M value */
            i=1; j=2;
            while ( j < M ) 
            {
                i=i+1;
                j=j*2;
            }

            //if ( j != M ) s_error("The number of QAM const points must be a power of 4" );
            //if ( i%2 != 0 ) s_error("The number of QAM const points must be a power of 4" );

            if ( j != M || i%2 != 0){
              it_warning("Non-square QAM are used" );
              //it_error("The number of QAM const points must be a power of 4" );

              if(M==32) APSK_32_modulation();
              else it_error("modulation not available");
              return;
            }

                /* compute sqrt M */
            lN = i/2;
            N=2;
            for ( j=1; j<lN; j++) N = N*2;
            
                /* allocate matrices */
            lab  = imat(N, N);
            bit1 = imat(N, N);
            bit2 = imat(N, N);
            
                /* init labels */
            for (i=0; i<N; i++) for(j=0; j<N; j++) lab(i,j) = 0;
            
                /* compute labels */
            pos = 1;
            for( s=0; s<lN; s++)
            {
				/* compute current bits */
                makebits(s, N, bit1, bit2);
                
				/* update labels */
                for (i=0; i<N; i++) for(j=0; j<N; j++) lab(i,j) += pos * bit1(i,j);
                pos = pos * 2;
                for (i=0; i<N; i++) for(j=0; j<N; j++) lab(i,j) += pos * bit2(i,j);
                pos = pos * 2;
            }
	    
                /* write labels */
            f = ( N - 1.) / 2.;
            for (i=0; i<N; i++) for(j=0; j<N; j++) 
	      modulation->SymbolMapping[ lab(i,j) ] = 
		complex<double>(double(i) -f, double(j) -f);
	    
                /* normalize energy */
            f = 0.0;
            for (i=0; i<M; i++)
                f += pow( modulation->SymbolMapping[i].real(), 2.0) +
		     pow( modulation->SymbolMapping[i].imag(), 2.0);
            f /= M; f = sqrt(f);

            for (i=0; i<M; i++)
            {
	        modulation->SymbolMapping[i]=
		  complex<double>(modulation->SymbolMapping[i].real() /f,
				 modulation->SymbolMapping[i].imag() /f);
            }

	    /* free mem */
	    //lab.~imat();
	    //bit1.~imat();
	    //bit2.~imat();
            
	    /* done */
            break;
	}
        default:
	    s_error("Unknown const type");
            break;
    }


    
    for(int j=0; j<M; j++){
	modulation->SymbolMapping[j] = complex<double>(
	    correct_rounding_error(modulation->SymbolMapping[j].real()),
	    correct_rounding_error(modulation->SymbolMapping[j].imag())
	    );
    }
    

    
#ifdef debug_cm
  cout<<modulation->SymbolMapping<<endl;
#endif
}


void CM::initModulation_GRAY()
{
    int i, j, k, M; 

    M = modulation->number_of_levels;
    
    switch( modulation->type )
    {
        case CPSK:
        {
            PSK my_psk(M);
	    bvec bits(modulation->bits_per_symbol*M);
	    
	    for(j=k=0;j<M;j++)
	      for(i=0; i<modulation->bits_per_symbol; i++) bits[k++] = (j>>i) & 1 ;
	    modulation->SymbolMapping = my_psk.modulate_bits( bits );

	    //my_psk.~PSK();
	    //bits.~bvec();
            break;
        }
        case CQAM:
        {
  	    QAM my_qam(M);
	    bvec bits(modulation->bits_per_symbol*M);
	    
	    for(j=k=0;j<M;j++)
	      for(i=0; i<modulation->bits_per_symbol; i++) bits[k++] = (j>>i) & 1 ;
	    modulation->SymbolMapping = my_qam.modulate_bits( bits );
	    
	    //my_qam.~QAM();
	    //bits.~bvec();	    
            break;
	}
        default:
	    s_error("Unknown const type");
            break;
    }

#ifdef debug_cm
    cout<<modulation->SymbolMapping<<endl;
#endif
}


cvec CM::modulate_symbols(ivec i_encoded_symbols)
{
  int i, N;
  cvec c_modulated;
  N = i_encoded_symbols.length();

  if(i_encoded_symbols.length()!=no_of_symbols){ 
    printf("i_encoded_symbols.length()=%d no_of_symbols=%d\n", i_encoded_symbols.length(), no_of_symbols);
    s_error("CM::modulate_symbols: make sure that i_encoded_symbols.length()==no_of_symbols ");
  }
  
  c_modulated.set_size(N, false);
  
  for(i=0;i<N;i++)
    c_modulated[i]=modulation->SymbolMapping[i_encoded_symbols[i]];
  
  if(interleaver->mode==IQ_INTLV){
    cvec i_c_modulated;
    //i_c_modulated.operator=(c_modulated);
    i_c_modulated=c_modulated;
    
    iq_intlvc( i_c_modulated, c_modulated, interleaver->length, interleaver->i_table, interleaver->q_table);
    
    //i_c_modulated.~cvec();
  }

#ifdef debug_cm  
  cout << "CM.modulate_symbols:" << endl << c_modulated<<endl; 
#endif

  return c_modulated;
}

void CM::demodulate_soft_symbols(cvec c_received_signals, double _2sigma2)
{// AWGN channel
  int k, i;
  double sum, dist;

  if(c_received_signals.length()!=no_of_symbols){ 
    printf("c_received_signals.length()=%d no_of_symbols=%d\n", c_received_signals.length(), no_of_symbols);
    s_error("CM::demodulate_soft_symbols: make sure that c_received_signals.length()==no_of_symbols ");
  }
  
  if(interleaver->mode==IQ_INTLV){
    demodulate_soft_symbols_IQ(c_received_signals, _2sigma2);
    return;
  }

  for ( k = 0; k < no_of_symbols; k++ ){
      for ( i = 0, sum=0; i < modulation->number_of_levels; i++ )
      {
	    dist = hypot ( (c_received_signals[k].real()-modulation->SymbolMapping[i].real()), 
			   (c_received_signals[k].imag()-modulation->SymbolMapping[i].imag()));
			   
	    Pr(k,i) = -(dist*dist) / (_2sigma2) ;
      }
  }
  //cout << Pr << endl;
}

void CM::demodulate_soft_symbols(cvec c_received_signals, cvec channel_gains, double _2sigma2)
{
  int k, i;
  double dist, x, y;

  if(c_received_signals.length()!=no_of_symbols || channel_gains.length()!=no_of_symbols){ 
    printf("channel_gains.length()=%d c_received_signals.length()=%d no_of_symbols=%d\n", 
	   channel_gains.length(), c_received_signals.length(), no_of_symbols);
    s_error("CM::demodulate_soft_symbols: make sure that channel_gains.length()==c_received_signals.length()==no_of_symbols ");
  }
  
  if(interleaver->mode==IQ_INTLV){
    demodulate_soft_symbols_IQ(c_received_signals, channel_gains, _2sigma2);
    return;
  }

  for ( k = 0; k < no_of_symbols; k++ ){
    for ( i = 0; i < modulation->number_of_levels; i++ )
    {
	x = modulation->SymbolMapping[i].real() * channel_gains[k].real()
	  - modulation->SymbolMapping[i].imag() * channel_gains[k].imag();
	y = modulation->SymbolMapping[i].real() * channel_gains[k].imag()
	  + modulation->SymbolMapping[i].imag() * channel_gains[k].real();
	
	dist = hypot ( c_received_signals[k].real()-x,
		       c_received_signals[k].imag()-y );
	
	Pr(k,i) = -(dist*dist) / (_2sigma2) ;
    }
  }
  
  //cout << Pr << endl;
}

// for cooperative diversity scheme: relay signals only
void CM::demodulate_soft_symbols_relay(cvec c_received_signals, cvec channel_gains, double N0, 
				       vec beta, vec var_n, vec pow_s, vec mu_n)
{
  int k, i;
  double x, y, alpha2, noise_variance, distance;
  vec signal(2), dist2(2), variance(2);
  cvec symbol(1);
  
  if(c_received_signals.length()!=no_of_symbols || channel_gains.length()!=no_of_symbols){ 
    printf("channel_gains.length()=%d c_received_signals.length()=%d no_of_symbols=%d\n", 
	   channel_gains.length(), c_received_signals.length(), no_of_symbols);
    s_error("CM::demodulate_soft_symbols: make sure that channel_gains.length()==c_received_signals.length()==no_of_symbols ");
  }
  
  if(interleaver->mode==IQ_INTLV) it_error("not ready yet");
  
  for ( k = 0; k < no_of_symbols; k++ ){
      c_received_signals[k] = conj(channel_gains[k]) * c_received_signals[k];      
      alpha2 = pow(abs(channel_gains[k]),2.);
      //cout<<channel_gains[k]<<endl;
      //cout<<alpha2<<endl; getchar();
      
      for ( i = 0; i < modulation->number_of_levels; i++ )
      {
	  signal[0] = modulation->SymbolMapping[i].real()*(1.-mu_n(0));
	  signal[1] = modulation->SymbolMapping[i].imag()*(1.-mu_n(1));
	  
	  //symbol[0] = complex<double>(alpha2*beta[0]*signal[0], alpha2*beta[1]*signal[1]);
	  //distance = abs(c_received_signals[k] - symbol[0]);
	  
	  dist2[0] = pow(abs(c_received_signals[k].real() - alpha2*beta[0]*signal[0]), 2);
	  dist2[1] = pow(abs(c_received_signals[k].imag() - alpha2*beta[1]*signal[1]), 2);
	  
	  variance[0] = alpha2*N0/2 + pow(alpha2*beta(0),2.)*var_n(0)*pow(modulation->SymbolMapping[i].real(),2.);
	  variance[1] = alpha2*N0/2 + pow(alpha2*beta(1),2.)*var_n(1)*pow(modulation->SymbolMapping[i].imag(),2.);
	  noise_variance = variance[0] + variance[1];
      	  
	  Pr(k,i) = -(dist2[0] + dist2[1]) / noise_variance ;	
	  
	  //Pr(k,i) = - pow(distance,2) / noise_variance ;	
      }
  }
  
  //cout << Pr << endl;
}

// for cooperative diversity scheme: perfect relay
void CM::demodulate_soft_symbols_relay(cvec c_received_signals, cvec channel_gains, double N0, 
				       cvec c_received_signals_rd, cvec channel_gains_rd, double N0_rd)
{
  int k, i;
  double x, y, alpha2, alpha2_rd, noise_variance, distance;
  vec signal(2), dist2(2), variance(2);
  cvec symbol(1);
  
  if(c_received_signals.length()!=no_of_symbols || channel_gains.length()!=no_of_symbols){ 
    printf("channel_gains.length()=%d c_received_signals.length()=%d no_of_symbols=%d\n", 
	   channel_gains.length(), c_received_signals.length(), no_of_symbols);
    s_error("CM::demodulate_soft_symbols: make sure that channel_gains.length()==c_received_signals.length()==no_of_symbols ");
  }
  
  if(interleaver->mode==IQ_INTLV) it_error("not ready yet");
  
  for ( k = 0; k < no_of_symbols; k++ ){
      // Maximal Ratio Combining
      c_received_signals[k] = conj(channel_gains[k]) * c_received_signals[k]
                          + conj(channel_gains_rd[k]) * c_received_signals_rd[k];
      
      alpha2    = pow(abs(channel_gains[k]),2.);
      alpha2_rd = pow(abs(channel_gains_rd[k]),2.);
      
      for ( i = 0; i < modulation->number_of_levels; i++ )
      {
	  
	  symbol[0] = (alpha2 + alpha2_rd)*modulation->SymbolMapping[i];
	  
	  dist2[0] = pow(c_received_signals[k].real() - symbol[0].real(), 2.);
	  dist2[1] = pow(c_received_signals[k].imag() - symbol[0].imag(), 2.);
	  
	  noise_variance = alpha2*N0 + alpha2_rd*N0_rd;
      	  
	  Pr(k,i) = -(dist2[0] + dist2[1]) / noise_variance;
	  
	  /*
	    // below gives the same result
	  symbol[0] = modulation->SymbolMapping[i];
	  
	  Pr(k,i) = -pow(abs(conj(channel_gains[k])*c_received_signals[k]-alpha2*symbol[0]),2.)/(alpha2*N0)
	            -pow(abs(conj(channel_gains_rd[k])*c_received_signals_rd[k]-alpha2_rd*symbol[0]),2.)/(alpha2_rd*N0_rd);
	  */
      }
  }
  
  //cout << Pr << endl;
}



// for cooperative diversity scheme: soft relay 
void CM::demodulate_soft_symbols_relay(cvec c_received_signals, cvec channel_gains, double N0, 
				       cvec c_received_signals_rd, cvec channel_gains_rd, double N0_rd,
				       vec beta, vec var_n, vec pow_s, vec mu_n)
{
    int k, m;
    int N = no_of_symbols;
    int M = modulation->number_of_levels;
    mat OPr(N,M);

    demodulate_soft_symbols(c_received_signals, channel_gains, N0);
    OPr = Pr;

    demodulate_soft_symbols_relay(c_received_signals_rd, channel_gains_rd, N0_rd,
				  beta, var_n, pow_s, mu_n);
    
    for(k=0;k<N;k++) for(m=0;m<M;m++) //combine two Pr
	Pr(k,m) += OPr(k,m);    
}

// for cooperative diversity scheme: Amplify And Forward (AAF)
void CM::demodulate_soft_symbols_relay_aaf(cvec c_received_signals, cvec channel_gains, double N0, 
				     cvec c_received_signals_rd, cvec channel_gains_rd, double N0_rd,
				     cvec channel_gains_sr, vec factor)
{
  int k, i;
  double dist, x, y, variance;

  if(c_received_signals.length()!=no_of_symbols || channel_gains.length()!=no_of_symbols){ 
    printf("channel_gains.length()=%d c_received_signals.length()=%d no_of_symbols=%d\n", 
	   channel_gains.length(), c_received_signals.length(), no_of_symbols);
    s_error("CM::demodulate_soft_symbols: make sure that channel_gains.length()==c_received_signals.length()==no_of_symbols ");
  }
  
  if(interleaver->mode==IQ_INTLV) it_error("not ready yet");
  
  //----------------------------
  mat OPr(no_of_symbols, modulation->number_of_levels);  
  demodulate_soft_symbols(c_received_signals, channel_gains, N0);
  OPr = Pr;
  //----------------------------
  
  for(k=0;k<no_of_symbols;k++)
      channel_gains[k] = factor[k] * channel_gains_rd[k] * channel_gains_sr[k];
  
  for ( k = 0; k < no_of_symbols; k++ ){
    for ( i = 0; i < modulation->number_of_levels; i++ )
    {
	x = modulation->SymbolMapping[i].real() * channel_gains[k].real()
	  - modulation->SymbolMapping[i].imag() * channel_gains[k].imag();
	y = modulation->SymbolMapping[i].real() * channel_gains[k].imag()
	  + modulation->SymbolMapping[i].imag() * channel_gains[k].real();
	
	dist = hypot ( c_received_signals_rd[k].real()-x,
		       c_received_signals_rd[k].imag()-y );
	
	variance = N0 + N0*pow(factor[k]*abs(channel_gains_rd[k]),2.);
	//Pr(k,i) = -(dist*dist) / variance ;
	Pr(k,i) = OPr(k,i) + (-(dist*dist)/variance);
    }
  }
  
  //cout << Pr << endl;    
    
}



void CM::demodulate_soft_symbols_IQ(cvec c_received_signals, double _2sigma2)
{// AWGN channel
  int k, i;
  double dist;
  
  for ( k = 0; k < no_of_symbols; k++ ){
      for ( i=0; i<modulation->number_of_I_levels; i++ ){
	dist = c_received_signals[k].real() - modulation->SymbolMappingI[i];
	PrI(k,i) = -(dist*dist) / (_2sigma2) ;
      }
      
      for ( i=0; i<modulation->number_of_Q_levels; i++ ){
	dist = c_received_signals[k].imag() - modulation->SymbolMappingQ[i];
	PrQ(k,i) = -(dist*dist) / (_2sigma2) ;
      }     
  }
  
  compute_Pr_from_PrIQ(Pr, PrI, PrQ);
  //cout << Pr << endl;
}

void CM::demodulate_soft_symbols_IQ(cvec c_received_signals, cvec channel_gains, double _2sigma2)
{
  int k, i;
  double dist, factor, received_transform_I, received_transform_Q;
  
  // received_transform = conjugate of channel_gains * c_received_signals
  for ( k = 0; k < no_of_symbols; k++ ){
      factor = channel_gains[k].real()*channel_gains[k].real() 
	     + channel_gains[k].imag()*channel_gains[k].imag();
      
      for ( i=0; i<modulation->number_of_I_levels; i++ ){
	received_transform_I =  c_received_signals[k].real()*channel_gains[k].real() 
	                       +c_received_signals[k].imag()*channel_gains[k].imag();
	
	dist = received_transform_I-factor*modulation->SymbolMappingI[i];
	PrI(k,i) = -(dist*dist) / (_2sigma2*factor) ;
      }
      
      for ( i=0; i<modulation->number_of_Q_levels; i++ ){
	received_transform_Q = -c_received_signals[k].real()*channel_gains[k].imag() 
	                       +c_received_signals[k].imag()*channel_gains[k].real();
	
	dist = received_transform_Q-factor*modulation->SymbolMappingQ[i];
	PrQ(k,i) = -(dist*dist) / (_2sigma2*factor) ;
      }     
  }
  
  compute_Pr_from_PrIQ(Pr, PrI, PrQ);
  //cout << Pr << endl <<Pr.rows<<endl<<Pr.cols<<endl;
}


/*----interleavers for CM------------------------------------------*/
static int i_count;

/* create odd-even separated Turbo TCM interleaver table */
void CM::init_TTCM_interleaver()
{
    i_count = 0;

    /* check */
    if( ttcm->interleaver->length < no_of_symbols )
        s_error("Increase your turbo interleaver size");
    
    /* init Odd-Even-Separated Turbo TCM pseudo-random interleaver*/    
    init_OES_interleaver(ttcm->interleaver->length,
                         ttcm->interleaver->table);
    
    /* find termination locations for 2nd encoder*/
    while( !intlv_termination_locat(ttcm->interleaver->length,
                                    tcm->L,
                                    ttcm->interleaver->table,
                                    tcm->last) );
    
    //cout << ttcm->interleaver->table << endl;
}

void CM::init_OES_interleaver(int n, ivec &i_table)
{
        int i, j, k;

        /* look for intlv file */
        {
                char txt1[256];
                FILE *fp;

                /* odd even separated interleaver for TTCM */
                sprintf( txt1, "interleaver_dir/Intlv.oes.%d", n );
                
                if ( NULL != (fp = fopen( txt1, "r" ) ) && (i_count==0) )
                {
                        /* read the intlv */
                        for ( i=0; i<n; i++ ) if ( 0 == fscanf( fp, "%d", &i_table[i] ) )
                            s_error( "Corrupted interleaver file " );
                        fclose ( fp );
                }
                else
                { 
                      /* make the intlv */
                      for (i=0; i<n; i++) i_table[i] = i;
                      
                      for (i=0; i<n; i=i+2 )
                      {       
                        /* select where to swap */
                        k = rand()%(n/2);
                        k = 2*k;
                        /* check */ 
                        while (k >= n ) k=k-2;
                        
                        /* swap */
                        j = i_table[k];
                        i_table[k] = i_table[i];
                        i_table[i] = j;
                      }
                      for (i=1; i<n; i=i+2 )
                      {       
                        /* select where to swap */
                        k = rand()%(n/2);
                        k = 2*k+1;
                        /* check */ 
                        while (k >= n ) k=k-2;
                        
                        /* swap */
                        j = i_table[k];
                        i_table[k] = i_table[i];
                        i_table[i] = j;
                      }

                      fp = fopen( txt1, "w" );
                      for ( i=0; i<n; i++ ) fprintf( fp, "%d\n", i_table[i] );
                      fclose( fp );
                      printf("Interleaver file for TTCM (%s) was created.\n", txt1 );
                 
                }
        }
        i_count ++;
}

/* find termination locations for 2nd encoder*/
/* L = memory length*/
int CM::intlv_termination_locat(int n, int L, ivec &i_table, ivec &last)
{
        int i, h, k;
        
        for( i=0; i<L; i++ ) last[i] = i_table[n-L+i];
        
        /* bubble sort */
        for( i=0; i<L; i++)
        {
                h = i;
                while( h>0 && last[h] < last[h-1] )
                {
                        k = last[h];
                        last[h] = last[h-1];
                        last[h-1] = k;
                        h--;
                }
        }

        /* a check */
        for ( i=0; i<L; i++ ) if( last[i] >= n-L ){
          printf("Interleaver not suitable for termination, make new one.\n");
          init_OES_interleaver(n, i_table);
          return 0;
        }

	//printf("\nTurbo TCM Interleaver created.\n");
        return 1;
}


void CM::intlvi( ivec x, ivec &y, int n, ivec i_table)
{
      int i;
      for( i=0; i<n; i++ ) y[i] = x[ i_table[i] ];
}


void CM::de_intlvi( ivec x, ivec &y, int n, ivec i_table)
{
      int i;
      for( i=0; i<n; i++ ) y[ i_table[i] ] = x[i];
}

void CM::intlvm_3d (mat *x, mat *&y, int n, ivec i_table)
{
      int i;

      for( i=0; i<n; i++ ){
	//y[i].operator=(x[ i_table[i] ]);
	y[i]=x[ i_table[i] ];
	//cout << y[i] << endl << x[i] << endl; getchar();
      }
}

void CM::de_intlvm_3d (mat *x, mat *&y, int n, ivec i_table)
{
      int i;

      for( i=0; i<n; i++ ){
	//y[i_table[i]].operator=(x[i]);
	y[i_table[i]]=x[i];
      }
}

void CM::intlvd_2d( mat x, mat &y, int n, ivec i_table)
{
      int i;

      for( i=0; i<n; i++ ){
	y.set_row(i, x.get_row( i_table[i] ));
	//cout<<y.get_row(i)<<x.get_row( i_table[i] ); getchar();
      }
      //cout << x << endl << y << endl; getchar();
}

void CM::de_intlvd_2d( mat x, mat &y, int n, ivec i_table)
{
      int i;
      for( i=0; i<n; i++ ){
	y.set_row(i_table[i], x.get_row(i) );
      }
      //cout << x << endl << y << endl;
}


/* Channel Interleaver */
void CM::init_channel_interleaver(int no_of_symbols)
{
    int i;
    char txt[256];
    
    //interleaver    = ( struct cm_interleaver_control *) calloc ( sizeof ( struct cm_interleaver_control), 1);
    interleaver    = new ( struct cm_interleaver_control);
    interleaver->mode = interleaver_mode;
    
    interleaver->length = no_of_symbols;	
    interleaver->table.set_size(interleaver->length, false);
    
    // initialise dummy symbol interleaver
    for(i=0; i<interleaver->length; i++)
      interleaver->table[i] = i;
    /*
    {
      FILE *fp;
      sprintf( txt, "Intlv.%d.dummy", interleaver->length );
      fp = fopen( txt, "w" );
      for ( i=0; i<interleaver->length; i++ ) fprintf( fp, "%d\n", interleaver->table[i] );
      fclose( fp );
    }
    //exit(0);
    */

    // initialise symbol interleaver 
    if(interleaver->mode==ON){
      sprintf( txt, "interleaver_dir/Intlv.%d", interleaver->length );
      init_random_interleaver(interleaver->length, interleaver->table, txt);	
    }
    else if(interleaver->mode==IQ_INTLV)
    {// initialise IQ interleaver       
	char i_txt[256];
	char q_txt[256];
	
	interleaver->length = no_of_symbols;
	
	interleaver->i_table.set_size(interleaver->length, false);              
	interleaver->q_table.set_size(interleaver->length, false);              

	sprintf( i_txt, "interleaver_dir/IntlvI.%d", interleaver->length );
	sprintf( q_txt, "interleaver_dir/IntlvQ.%d", interleaver->length );
    
	init_random_interleaver(interleaver->length, interleaver->i_table, i_txt);
	init_random_interleaver(interleaver->length, interleaver->q_table, q_txt);
	
	init_iq_tables();
	PrI  = mat( no_of_symbols, modulation->number_of_I_levels);
	PrQ  = mat( no_of_symbols, modulation->number_of_Q_levels);
    }
    
    if(mode==BICM || mode==BICMID)
    {// initialise parallel bit interleavers         
        static int done=0;
        int max_intlv=8;
	
	if(bicm->n>8)
	{ 
	  max_intlv=bicm->n;
	  if(done==0){ 
	    printf("rm -f interleaver_dir/Bit*Intlv.*\n");
	    system("rm -f interleaver_dir/Bit*Intlv.*");
	    done=1;
	  }
	}

        //bicm->bit_interleaver = (struct  cm_interleaver_control ** ) calloc ( sizeof ( struct cm_interleaver_control *), max_intlv);
        //bicm->bit_interleaver = new (struct cm_interleaver_control *)[max_intlv];
	bicm->bit_interleaver = new struct cm_interleaver_control * [max_intlv];
	
	for(i=0; i<max_intlv; i++)
	{
 	    //bicm->bit_interleaver[i] = ( struct cm_interleaver_control *) calloc ( sizeof ( struct cm_interleaver_control ), 1 );
	    bicm->bit_interleaver[i] = new ( struct cm_interleaver_control);
	    
	    bicm->bit_interleaver[i]->length = no_of_symbols;
	    bicm->bit_interleaver[i]->table.set_size(bicm->bit_interleaver[i]->length, false);
	    
	    sprintf( txt, "interleaver_dir/Bit%dIntlv.%d", i, bicm->bit_interleaver[i]->length );
	    init_random_interleaver(bicm->bit_interleaver[i]->length,
				    bicm->bit_interleaver[i]->table, txt); 	  
	}
    }    
}

/* Free Channel Interleaver */
void CM::free_channel_interleaver()
{
    int i;
    
    if(interleaver!=NULL)
    {
        if(interleaver->mode==IQ_INTLV){
	  interleaver->i_table.set_size(0,false);
	  interleaver->q_table.set_size(0,false);
	  PrI.set_size(0,false);
	  PrQ.set_size(0,false);
	}
	
	interleaver->table.set_size(0,false);
	//free(interleaver);
	delete interleaver;
    }
    
    if(mode==BICM || mode==BICMID)
    {
        int max_intlv=8;	
	if(bicm->n>8) max_intlv=bicm->n;
	
	for(i=0; i<max_intlv; i++){
	  if(bicm->bit_interleaver[i]!=NULL){
	    bicm->bit_interleaver[i]->table.set_size(0,false);	    
	    //free(bicm->bit_interleaver[i]); 	    
	    delete bicm->bit_interleaver[i]; 	    
	  }
	}
	
	if(bicm->bit_interleaver!=NULL)	delete [] bicm->bit_interleaver;
    } 
}

void CM::init_random_interleaver(int n, ivec &table, char *txt)
{
    int i,j,k;
    FILE *fp;
    
    if ( n <= 0 ) s_error( "Bad interleaver dimension");
    
    if ( NULL != (fp = fopen( txt, "r" ) ) )
    {
            /* read the intlv */
        for ( i=0; i<n; i++ ) if ( 0 == fscanf( fp, "%d", &table[i] ) )
            s_error( "Corrupted interleaver file " );
        fclose ( fp );
    }
    else
    {  /* make the intlv */
        for (i=0; i<n; i++) table[i] = i;
        
        for (i=0; i<n; i++){    
                k = rand()%n;
                j = table[k];
                table[k] = table[i];
                table[i] = j;
        }
                  
        fp = fopen( txt, "w" );
        for ( i=0; i<n; i++ ) fprintf( fp, "%d\n", table[i] );
        fclose( fp );
        printf("Interleaver file (%s) was created\n", txt );        
    }
}

void CM::bit_intlvi( imat x, imat &y, int block_length, ivec i_table, int index)
{
      int i;      
      for( i=0; i<block_length; i++ ) y(index,i) = x(index, i_table[i] );
}

void CM::bit_de_intlvi( imat x, imat &y, int block_length, ivec i_table, int index)
{
      int i;      
      for( i=0; i<block_length; i++ ) y(index,i_table[i]) = x(index, i);
}

void CM::bit_intlvm_3d (mat * x, mat *&y, int n, int m, ivec i_table, int index)
{
      int i,j;
      
      for( i=0; i<n; i++ ) for( j=0; j<m; j++ ) 
	y[index]( i, j) = x[index](i_table[i], j);
}

void CM::bit_de_intlvm_3d (mat * x, mat *&y, int n, int m, ivec i_table, int index)
{
      int i,j;
      
      for( i=0; i<n; i++ ) for( j=0; j<m; j++ ) 
	y[index]( i_table[i], j) = x[index](i,j);
}

void CM::iq_intlvc( cvec x, cvec &y, int n, ivec i_table, ivec q_table) 
{
      int i;
      
      for( i=0; i<n; i++ )
	y[i] = complex<double>(x[i_table[i]].real(), x[q_table[i]].imag());
}

void CM::iq_de_intlvc( cvec x, cvec &y, int n, ivec i_table, ivec q_table) 
{
      int i;
      vec Re(n), Im(n);
      
      for( i=0; i<n; i++ ){
	Re[i_table[i]] = x[i].real();
	Im[q_table[i]] = x[i].imag();
      }
      
      for( i=0; i<n; i++ )
	y[i] = complex<double>(Re[i], Im[i]);
}

void CM::compute_Pr_from_PrIQ(mat &Pr, mat PrI, mat PrQ)
{
    int loop, i, j, k;
    int N = Pr.rows();
    
    for ( k=0; k<N ; k++ )
      for ( loop=0; loop < modulation->number_of_levels; loop++ )
	{
	  for ( i=0; i<modulation->number_of_I_levels; i++ )
	    if(modulation->SymbolMappingI[i] == modulation->SymbolMapping[loop].real()) break;
	  
	  for ( j=0; j<modulation->number_of_Q_levels; j++ )
	    if(modulation->SymbolMappingQ[j] == modulation->SymbolMapping[loop].imag()) break;
	  
	  Pr(k,loop) =  PrI(k,i) + PrQ(k,j);
	}

    //printf("compute_Pr_from_PrIQ:\n"); cout << Pr.get_row(99); getchar();
}

void CM::get_I_table()
{
  int i,k,New, tot_idx;
  double temp;
  int M = modulation->number_of_levels;

  modulation->SymbolMappingI.set_size(2);  
  
  //-----------find total no. of different values
  for(k=0,tot_idx=1,modulation->SymbolMappingI[0]=modulation->SymbolMapping[0].real(); k<M; k++){
      for(i=0,New=1; i<tot_idx; i++)
          if( modulation->SymbolMapping[k].real() == modulation->SymbolMappingI[i])
            {New=0; break;} 
        
      if(New==1){
	  modulation->SymbolMappingI.ins(tot_idx,modulation->SymbolMapping[k].real());
	  //modulation->SymbolMappingI[tot_idx]= modulation->SymbolMapping[k].real();
          tot_idx++;
      }
  }
  modulation->SymbolMappingI.del(tot_idx);
  modulation->number_of_I_levels = tot_idx;
  
  //-----------sort the values
  for(k=0,i=1; k<tot_idx;){
    //printf("\nk%d i%d",k,i);
    if( (modulation->SymbolMappingI[k] > modulation->SymbolMappingI[i]) ){
      temp = modulation->SymbolMappingI[i]; 
      modulation->SymbolMappingI[i] = modulation->SymbolMappingI[k]; 
      modulation->SymbolMappingI[k] = temp;
      i++;
    }
    else i++;
    
    if(i>=tot_idx){ 
      if( ( (++k)==tot_idx) || ( (k+1)==tot_idx) ) break;
      i=k+1;
    }
  }
  //for(i=0; i<tot_idx; i++) printf("\nI : %d %f",i, modulation->SymbolMappingI[i]); getchar();
}

void CM::get_Q_table()
{
  int i,k,New, tot_idx;
  double temp;
  int M = modulation->number_of_levels;
  
  modulation->SymbolMappingQ.set_size(2);  
  
  //-----------find total no. of different values
  for(k=0,tot_idx=1,modulation->SymbolMappingQ[0]=modulation->SymbolMapping[0].imag(); k<M; k++){
      for(i=0,New=1; i<tot_idx; i++)
          if( modulation->SymbolMapping[k].imag() == modulation->SymbolMappingQ[i])
            {New=0; break;} 
      
      if(New==1){
	  modulation->SymbolMappingQ.ins(tot_idx,modulation->SymbolMapping[k].imag());
          modulation->SymbolMappingQ[tot_idx]= modulation->SymbolMapping[k].imag();
          tot_idx++;
      }
  }
  modulation->SymbolMappingQ.del(tot_idx);  
  modulation->number_of_Q_levels = tot_idx;
  
  //-----------sort the values
  for(k=0,i=1; k<tot_idx;){
    //printf("\nk%d i%d",k,i);
    if( (modulation->SymbolMappingQ[k] > modulation->SymbolMappingQ[i]) ){
      temp = modulation->SymbolMappingQ[i]; 
      modulation->SymbolMappingQ[i] = modulation->SymbolMappingQ[k]; 
      modulation->SymbolMappingQ[k] = temp;
      i++;
    }
    else i++;

    if(i>=tot_idx){ 
      if( ( (++k)==tot_idx) || ( (k+1)==tot_idx) ) break;
      i=k+1;
    }
  }
  //for(i=0; i<tot_idx; i++) printf("\nQ : %d %f",i, modulation->SymbolMappingQ[i]); getchar();
}

void CM::init_iq_tables()
{
      int i;
      
      for(i=0; i<modulation->number_of_levels; i++){
        modulation->SymbolMapping[i]=
	  complex<double>(correct_rounding_error(modulation->SymbolMapping[i].real()),
			 correct_rounding_error(modulation->SymbolMapping[i].imag()));
      }
      
      get_I_table();
      get_Q_table();

#ifdef debug_cm  
      cout<<modulation->SymbolMapping<<endl;
      cout<<modulation->SymbolMappingI<<endl;
      cout<<modulation->SymbolMappingQ<<endl;
#endif
      
}



//-----------------------------------------

void CM::s_error ( const char * message )
{
    fprintf (stderr, "Fatal: %s\n", message);
    fflush ( stderr );
    exit (-1);
}

void CM::makebits(int s, int N, imat &bit1, imat &bit2)
{
    int i, j;
    int p; 
    imat v;
    
    v = imat(2, N);
    p=1; for( i=0; i<s; i++) p = p*2;
    
	/* make vectors */
    for ( i=0; i<N; i++ ) { v(0,i) = ( i/p ) % 2; v(1,i) = 1 - v(0,i); }
    
	/* make bit matrices */
    for ( i=0; i<N; i++ ) for ( j=0; j<N; j++ )
    { 
        bit1(i,j) = v( (i/p)%2  ,j);
        bit2(i,j) = v(0,j);
    }
    
	/* done */
    //v.~imat();
}

double CM::correct_rounding_error(double distance){
  // to the precision of 7 decimal places
  double dist;
  double sign, magnitude;

  if(distance<0) sign = -1;
  else           sign =  1;

  magnitude = abs(distance);

  dist  = floor(magnitude * 1.0e9);
  dist /= 1.0e9;

  dist  = ceil(dist * 1.0e7);
  dist /= 1.0e7;

  return (dist * sign);
}

/*
double CM::correct_rounding_error(double distance){
  // to the precision of 6 decimal places
  double dist;    
  dist  = floor(distance * 1.0e8);    
  dist /= 1.0e8;
  
  dist  = ceil(dist * 1.0e6);    
  dist /= 1.0e6;
  
  return (dist);
}
*/

mat CM::get_normal_domain_OPr(void){
    int N=no_of_symbols;
    int M=modulation->number_of_levels;
    mat normal_OPr(N,M);
    int k, m;
    double sum;
    for(k=0;k<N;k++){
	sum = 0;
	for(m=0;m<M;m++){
	    normal_OPr(k,m) = exp(OPr(k,m));
	    sum += normal_OPr(k,m);
	}
	for(m=0;m<M;m++) normal_OPr(k,m) /= sum;
    }
    
    return normal_OPr;
}


/* Octal to decimal */
int CM::octal_to_decimal(int gen){
  
  if(gen<10) return gen;
  else if(gen<100){
    return(8*(gen/10) + gen%10);
  }
  else if(gen<1000){    
    return(64*(gen/100) + 8*( (gen%100)/10 ) + gen%10);
  }
  else if(gen<10000){
    return(512*(gen/1000) + 64*( (gen%1000)/100 ) + 8*( (gen%100)/10 ) + gen%10);
  }
  else s_error("Octal to decimal not complete for octal number>10000.");

  return 0;
}

/*------ jacobian logarithm ------------------ */
double jacolog_1( double x, double y)
{
    double r;
    
    if(x>y) r = x + log ( 1 + exp( y - x ) );
    else    r = y + log ( 1 + exp( x - y ) );

    return r;
}

double jacolog_2( double x, double y)
{       
        double r;
        double diff;

        if(x>y){ r = x; diff=x-y; }
        else   { r = y; diff=y-x; }  
              
        if(diff > 3.7 )      r += 0.00;
        else if(diff > 2.25) r += 0.05;
        else if(diff > 1.5 ) r += 0.15;
        else if(diff > 1.05) r += 0.25;
        else if(diff > 0.7 ) r += 0.35;
        else if(diff > 0.43) r += 0.45;
        else if(diff > 0.2 ) r += 0.55;
        else                 r += 0.65;
        
        return r;
}

double jacolog_3( double x, double y)
{
    double r;
    
    if(x>y) r = x;
    else    r = y;
    
    return r;
}

double CM::jacolog(double x, double y)
{
  if(decoder_type==CM_EXACT_LOG_MAP) return jacolog_1(x, y);
  else if(decoder_type==CM_APPROX_LOG_MAP) return jacolog_2(x, y);
  else if(decoder_type==CM_MAX_LOG_MAP) return jacolog_3(x, y);
  else s_error("unknown decoder type");
}


/*-----------symbol and bits probabilities conversion in log domain------------------*/
void CM::Pr_to_BitPr_log(int N, int lM, mat Pr, mat *&BitPr, int frame_index){
    int k, m, i, M;
    double max;

    M = (int)pow(2, lM);
    
    for(k=0;k<N;k++){
        for(i=0;i<lM;i++){

            max=MINF; BitPr[i](k + frame_index*N, 0) = BitPr[i](k + frame_index*N, 1) = MINF;
            for(m=0;m<M;m++)
	      BitPr[i](k + frame_index*N, (m>>i)&1 ) = jacolog( BitPr[i](k + frame_index*N, (m>>i)&1 ) , Pr(k,m));
            
            if( BitPr[i](k + frame_index*N, 0) < BitPr[i](k + frame_index*N, 1) ) max = BitPr[i](k + frame_index*N,1);
            else max = BitPr[i](k + frame_index*N,0);
            
            for(m=0;m<2;m++) BitPr[i](k + frame_index*N,m) -= max;
        }
    }
}

void CM::BitPr_to_Pr_log(int N, int lM, mat &Pr, mat *BitPr, int frame_index){
    int k, m, i, M;
    double max;

    M = (int)pow(2, lM);
  
    for(k=0;k<N;k++){
        max = MINF;
        for(m=0;m<M;m++){
            Pr(k,m) = 0; /*log 1 = 0*/
            for(i=0;i<lM;i++){
                Pr(k,m) += BitPr[i](k + frame_index*N, (m>>i)&1 );
            }

            if(max < Pr(k,m)) max = Pr(k,m);
        }
        for(m=0;m<M;m++) Pr(k,m) -= max;
        //        for(m=0;m<M;m++){ if(Pr[k][m] <= MINF) Pr[k][m]=MINF; }
    }
}

/*-----------symbol and bits conversion------------------*/
void CM::symbol_to_bits(int word_length, int block_length, ivec symbols, imat &bits_block){
  int i, j;
  
  for(j=0; j<block_length; j++)
      for(i=0; i<word_length; i++)
          bits_block(i,j) = (symbols[j]>>i) & 1 ;
}

void CM::bits_to_symbol(int word_length, int block_length, imat bits_block, ivec &symbols){
  int i, j;
  
  for(j=0; j<block_length; j++)
      for(i=0,symbols[j]=0; i<word_length; i++)
          symbols[j] += bits_block(i,j) * (1 << i);
}

ivec CM::symbol_to_bits_seq(int word_length, int block_length, ivec symbols){
  int i, j, k;
  int N=block_length*word_length;
  ivec bits_seq;
  bits_seq.set_length(N, false);

  if(N!=bits_seq.length()){ 
    printf("%d %d ",N, bits_seq.length());
    s_error("symbol_to_bits_seq: check bits_seq.length()");
  }
  
  for(k=j=0; j<block_length; j++)
      for(i=0; i<word_length; i++)
          bits_seq[k++] = (symbols[j]>>i) & 1 ;

  return bits_seq;
}

ivec CM::bits_seq_to_symbol(int word_length, int block_length, ivec bits_seq){
  int i, j, k;
  int N=block_length*word_length;
  ivec symbols;
  symbols.set_length(block_length, false);

  if(N!=bits_seq.length()){
    printf("%d %d ",N, bits_seq.length());
    s_error("bits_seq_to_symbol: check bits_seq.length()");
  }

  for(j=k=0; j<block_length; j++)
    for(i=0,symbols[j]=0; i<word_length; i++){
          symbols[j] += bits_seq[k++] * (1 << i);
	  //printf("%d(%d) ",bits_seq[k-1], symbols[j]);
    }
  
  return symbols;
}



