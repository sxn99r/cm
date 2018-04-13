/*! 
  \file
  \brief Definition of Coded Modulation (CM) class
  \author Michael Ng

  1.03
  
  2009/06/24
*/

#ifndef _CM_H_
#define _CM_H_

#include "itpp/itbase.h"
#include "cm.h"

/*! \defgroup coded_modulation Coded Modulation
 */

using namespace itpp;
using namespace std;


//#define debug_cm

//! define CM-based Phase-Shift-Keying modulation
#define CPSK    0
//! define CM-based Quadrature-Amplitude-Modulation
#define CQAM    1

//! define Gray-coded (GRAY) signal labelling 
#define GRAY    0
//! define Set Partition (SP) based signal labelling 
#define SP      1

//! define OFF
#define OFF      0
//! define ON
#define ON       1
//! define In-phase Quadrature-phase Interleaving 
#define IQ_INTLV 2

//! define Trellis Coded Modulation
#define TCM    1
//! define Turbo Trellis Coded Modulation
#define TTCM   2
//! define Bit-Interleaved Coded Modulation
#define BICM   3
//! define Bit-Interleaved Coded Modulation with Iterative Decoding
#define BICMID 4

//! define Exact Logarithmic (Log) Maximum A-posteriori Probability (MAP) decoding
#define CM_EXACT_LOG_MAP  1
//! define Approximate Log MAP decoding
#define CM_APPROX_LOG_MAP 2
//! define Maximum Log MAP decoding
#define CM_MAX_LOG_MAP    3

//! define Minimum Log probability (-infinity)
#define MINF   -100000
//! define No Linkage for a particular trellis transition
#define NO_LINK 100000

//! struct of CM modulator
struct modulation_param
{
  int  mode; //!< modulator mode: \ref GRAY, \ref SP
  int  type; //!< modulator type: \ref CPSK, \ref CQAM
  int  bits_per_symbol;  //!< number of modulated bits per symbol
  int  number_of_levels; //!< number of levels of modulated complex-symbol
  cvec SymbolMapping;    //!< Mapping of real-symbol index and modulated complex-symbol
  
  int  number_of_I_levels; //!< for IQ-interleaver: number of In-phase         levels of modulated symbol
  int  number_of_Q_levels; //!< for IQ-interleaver: number of Quadrature-phase levels of modulated symbol
  vec  SymbolMappingI;     //!< for IQ-interleaver: Mapping of real-symbol index and I of modulated symbol
  vec  SymbolMappingQ;     //!< for IQ-interleaver: Mapping of real-symbol index and Q of modulated symbol
};

//! struct of CM interleaver
struct cm_interleaver_control
{
  int    mode;    //!< interleaver mode: \ref OFF, \ref ON, \ref IQ_INTLV
  int    length;  //!< interleaver length 
  ivec   table;   //!< interleaver table
  
  ivec   i_table; //!< for IQ-interleaver: I interleaver table
  ivec   q_table; //!< for IQ-interleaver: Q interleaver table
};

/*!
  \ingroup coded_modulation  
  \brief struct of TCM for CM
*/
struct TCM_parameters
{
  int k;          //!< length of dataword
  int n;          //!< length of codeword
  int L;          //!< code memory length
    
  ivec last;      //!< termination location of terminated codewords
  
  int NoBranches; //!< number of possible output trellis branches
  int NoStates;   //!< number of possible trellis states 
  
  ivec GenPoly;   //!< generator polynomials
  imat Lb;        //!< codeword (trellis transition Label) table
  imat Ns;        //!< Next state table
  imat Ps;        //!< Previous state table
};

/*!
  \ingroup coded_modulation  
  \brief struct of TTCM for CM: consider identical component codes
*/
struct TTCM_parameters
{
  int iterations;      //!< number of turbo iterations     
  struct cm_interleaver_control *interleaver;  //!< turbo interleaver
};

/*!
  \ingroup coded_modulation  
  \brief struct of BICM for CM: rate-5/6 64QAM BICM employs puncturing on rate-1/2 mother code
*/
struct BICM_parameters
{
  int k;          //!< length of dataword
  int n;          //!< length of codeword
  int L;          //!< code memory length
  int M;          //!< code memory length for individual databit
  
  int puncture_code; //!< apply puncturing code: OFF, ON
  ivec puncture;     //!< puncturing pattern for 1/2-rate mother code
  
  struct cm_interleaver_control **bit_interleaver;  //!< parallel bit interleavers
  
  int NoBranches; //!< number of possible output trellis branches
  int NoStates;   //!< number of possible trellis states 
  
  imat GenPoly;   //!< generator polynomiala
  imat Lb;        //!< codeword (trellis transition Label) table
  imat Ns;        //!< Next state table
  imat Ps;        //!< Previous state table
};

/*!
  \ingroup coded_modulation  
  \brief struct of BICMID for CM: employing soft demodulator feedback
*/
struct BICMID_parameters
{
    int iterations;      //!< number of turbo iterations
};

//! struct of MAP decoder: for TCM and TTCM in CM
struct MAP_dec
{
  mat Apr;             //!< a-priori & extrinsic probability of dataword
  mat Apo;             //!< a-posteriori         probability of dataword

  mat Extr;            //!< extrinsic            probability of dataword
  
  mat *Ip1;            //!< 1st TCM decoder: channel symbol probability
  mat *Ip2;            //!< 2nd TCM decoder: channel symbol probability
};

//! struct of Soft-In Soft-Out decoder: for BICM and BICMID in CM
struct SISO_dec 
{
  mat Apo_dataword;    //!< a-posteriori probability of dataword
  mat *Apo_data_bit;   //!< a-posteriori probability of databit
  mat *Apr_coded_bit;  //!< a-priori & extrinsic probability of coded-bit 
  mat *Apo_coded_bit;  //!< a-posteriori probability of coded-bit 
};

/*! 
  \ingroup coded_modulation  
  \brief CM Class
  
  This class provides four coded modulation schemes: 
  - \ref TCM, \ref TTCM, \ref BICM and \ref BICMID.
  
  Modulation modes available are M-QAM and M-PSK, where \a M=2^n is the modulation level. We have:
  - 4-QAM, 16-QAM, 64-QAM (\ref CQAM) 
  - 4-PSK, 8-PSK, 16-PSK, 64-PSK (\ref CPSK)

  Interleaver types available are:
  - symbol \a interleaver (\ref ON, \ref OFF) for TCM, TTCM
  - internal parallel bit interleavers for \ref BICM and \ref BICMID (always \ref ON for \ref BICM & \ref BICMID)
  - IQ \a interleaver (\ref IQ_INTLV) for TCM, TTCM, BICM, BICMID
  
  Decoding in logarithmic domain is performed: \a decoder_type
  available are \ref CM_EXACT_LOG_MAP, \ref CM_APPROX_LOG_MAP, \ref CM_MAX_LOG_MAP.
  
  To set up a rate-3/4 (\a k=3, \a n=4) 16QAM TTCM having code memory
  3, code termination, 4 turbo iteration, \ref CM_APPROX_LOG_MAP decoder, 100
  coded symbols (\a no_of_symbols), employ IQ-channel interleaver; the
  following code can be used:
  \code
  CM cm;
  cm.set_parameters(TTCM,3,4,3,ON,4,CM_APPROX_LOG_MAP,100,CQAM,IQ_INTLV);
  cm.initialise();
  \endcode
  See cm_test.cpp for more details.

  \warning 
  - Modulation level = \a 2^n
  - Make sure that \a n=k+1; so that when compared CM to uncoded scheme
    -# the modulation level is doubled but 
    -# experience no bandwidth expansion
  - \a no_of_info_bits=no_of_info_symbols*k, where
    -# \a no_of_info_symbols=no_of_symbols     \n if \a Terminated=OFF for all CM schemes
    -# \a no_of_info_symbols=no_of_symbols-L   \n if \a Terminated=ON for TCM, BICM & BICMID
    -# \a no_of_info_symbols=no_of_symbols-2*L \n if \a Terminated=ON for TTCM
*/
class CM
{
 public:
  //! Class constructor
  CM(){}
  //! Destructor
  ~CM();

  /*! 
    \brief Setup parameters for the CM encoder/decoder and also calculate the bit and symbol block lengths based on a control file
    
    \param fname the control file name.

    \note after reading the parameters from a control file, set_parameters() will be called.
  */  
  void set_parameters_from_file(char *fname);
  
  /*! 
    \brief Setup parameters for the CM encoder/decoder and also calculate the bit and symbol block lengths
    
    \param mode_in CM.mode: \ref TCM, \ref TTCM, \ref BICM, \ref BICMID
    \param k_in    dataword length
    \param n_in    codeword length
    \param L_in    code memory length
    \param Terminated_in        code termination: \ref OFF, \ref ON
    \param iterations_in        turbo iteration number, for \ref TTCM and \ref BICMID
    \param decoder_type_in      decoding type: \ref CM_EXACT_LOG_MAP, \ref CM_APPROX_LOG_MAP, \ref CM_MAX_LOG_MAP
    \param no_of_symbols_in     number of coded symbols in a block 
    \param modulation_type_in   modulator type: \ref CPSK, \ref CQAM
    \param interleaver_mode_in  interleaver mode: \ref OFF, \ref ON, \ref IQ_INTLV (internal bit interleaver for \ref BICM or \ref BICMID is always \ref ON)    
    
    \warning
    - Modulation level = \f$ 2^{n\_in} \f$
    - Make sure that \a n_in=k_in+1; so that when compared CM to uncoded scheme
      -# the modulation level is doubled but 
      -# experience no bandwidth expansion
    
    \note
    - \a k=k_in, \a n=n_in, \a L=L_in 
    - if \a Terminated is OFF: 
      -# \a no_of_info_symbols=no_of_symbols_in
    - if \a Terminated is ON: 
      -# \a no_of_info_symbols=no_of_symbols_in-2*L (TTCM) 
      -# \a no_of_info_symbols=no_of_symbols_in-L (TCM, BICM & BICMID) 
    - \a no_of_info_bits=no_of_info_symbols*k 
    - \a no_of_coded_bits=no_of_symbols*n
    - \a no_of_tail_bits=(no_of_symbols-no_of_info_symbols)*k
  */  
  void set_parameters(const int mode_in, const int k_in, const int n_in, const int L_in, const int Terminated_in, 
		      const int iterations_in, const int decoder_type_in, const int no_of_symbols_in, 
		      const int modulation_type_in, const int interleaver_mode_in);

  /*!
    \brief reset symbol block length and interleaver for variable length coded modulation
    
    \param  new_no_of_symbols new symbol block length ( <= \a max_no_of_symbols).
  */
  void reset_CM(int new_no_of_symbols);

  /*!
    \brief reset number of turbo iterations for variable-iterated TTCM and BICMID
    
    \param  new_iterations new number of turbo iterations
  */
  void reset_CM_iteration(int new_iterations);
  
  /*!
    \brief Initialise CM: allocate memory and initialise coder, modulator (\a modulation ) & \a interleaver
    
    \note
    - \a tcm , \a ttcm and \a bicmid employ \ref SP based signal mapping
    - \a bicm employs \ref GRAY coded signal mapping
    - interleaver tables will be generated and stored in files.
  */
  void initialise();
  
  /*! 
    \brief CM encoder: encode and channel interleave
    
    \param b_Input_bits     input data bits (binary vector) 
    \param i_Output_symbols output coded symbol (integer vector)
    
    \note
    - length of \a b_Input_bits=no_of_info_symbols*k
    - length of \a i_Output_symbols=no_of_symbols
  */ 
  int encode ( bvec b_Input_bits, ivec &i_Output_symbols);
  
  /*! 
    \brief CM encoder: encode and channel interleave
    
    \param b_Input_bits     input data bits (binary vector) 
    \param i_Output_bits    output coded bits (integer vector)
    
    \note
    - length of \a b_Input_bits=no_of_info_symbols*k
    - length of \a i_Output_bits=no_of_symbols*n
  */ 
  int encode_bits ( bvec b_Input_bits, ivec &i_Output_bits);
  
  /*! 
    \brief CM decoder: channel deinterleave and decode
    
    \param b_decoded_bits decoded bits (binary vector) 
    
    \note 
    - data symbol a-posteriori probability updated and stored in 
      -# map->Apo (TCM, TTCM)
      -# siso->Apo_dataword (BICM, BICMID)
    - coded symbol a-posteriori probability updated and stored in 
      -# OPr (TCM, TTCM)
    - coded bit a-posteriori-probability updated and stored in 
      -# siso->Apo_coded_bit (BICM, BICMID)      
  */   
  int decode( bvec &b_decoded_bits);

  /*! 
    \brief CM decoder: channel deinterleave and decode
    
    \param b_decoded_bits decoded bits (binary vector) 
    \param cm_apr_databits_llr a-priori LLR of databits of CM scheme
    \return a-posteriori LLR of databits
    
    \note 
    - data symbol a-posteriori probability updated and stored in 
      -# map->Apo (TCM, TTCM)
      -# siso->Apo_dataword (BICM, BICMID)
    - coded symbol a-posteriori probability updated and stored in 
      -# OPr (TCM, TTCM)
    - coded bit a-posteriori-probability updated and stored in 
      -# siso->Apo_coded_bit (BICM, BICMID)      
  */
  vec decode( bvec &b_decoded_bits, vec cm_apr_databits_llr);
  
  /*! 
    \brief CM decoder: channel deinterleave and decode
    
    \param b_decoded_bits decoded bits (binary vector) 
    \param Pr matrix of the channel symbol probability 

    In this function, the CM's demodulator is not used to calculate the  
    channel symbol probability. Useful when the equaliser/space-time decoder was used
    to calculate the channel symbol probability.
    \note 
    - data symbol a-posteriori probability updated and stored in 
      -# map->Apo (TCM, TTCM)
      -# siso->Apo_dataword (BICM, BICMID)
    - coded symbol a-posteriori probability updated and stored in 
      -# OPr (TCM, TTCM)
    - coded bit extrinsic-probability updated and stored in 
      -# siso->Apo_coded_bit (BICM, BICMID)      
  */   
  int decode_using_Pr( bvec &b_decoded_bits, mat Pr);

  /*! 
    \brief CM decoder: channel deinterleave and decode
    
    \param b_decoded_bits decoded bits (binary vector) 
    \param PrI matrix of the channel symbol probability 
    \param PrQ matrix of the channel symbol probability 

    In this function, the CM's demodulator is not used to calculate the  
    channel symbol probability. Useful when the equaliser/space-time decoder was used
    to calculate the channel symbol probability. 
    This function is used for the IQ-interleaved system.
    \note 
    - data symbol a-posteriori probability updated and stored in 
      -# map->Apo (TCM, TTCM)
      -# siso->Apo_dataword (BICM, BICMID)
    - coded symbol a-posteriori probability updated and stored in 
      -# OPr (TCM, TTCM)
    - coded bit extrinsic-probability updated and stored in 
      -# siso->Apo_coded_bit (BICM, BICMID)      
  */
  int decode_using_PrIQ( bvec &b_decoded_bits, mat PrI, mat PrQ);
  
  /*! 
    \brief CM modulator
    
    \param i_encoded_symbols coded symbol (integer vector)
    \return  modulated symbol (complex vector)
  */
  cvec modulate_symbols(ivec i_encoded_symbols);

  /*! 
    \brief CM demodulator for AWGN channel
    
    \param c_received_signals received symbol (complex vector)
    \param _2sigma2           2*sigma^2
    
    \note channel symbol probability CM::Pr (double matrix) updated
  */
  void demodulate_soft_symbols(cvec c_received_signals, double _2sigma2);


  /*! 
    \brief CM demodulator for flat Rayleigh fading channel
    
    \param c_received_signals received symbol (complex vector)
    \param channel_gains      channel gains (complex vector)
    \param _2sigma2           2*sigma^2
    
    \note channel symbol probability CM::Pr (double matrix) updated
  */
  void demodulate_soft_symbols(cvec c_received_signals, cvec channel_gains, double _2sigma2);


  /*! 
    \brief CM demodulator for flat Rayleigh fading channel in cooperative scheme: relay signals only
    
    \param c_received_signals received symbol (complex vector)
    \param channel_gains      channel gains (complex vector)
    \param _2sigma2           2*sigma^2
    
    \note channel symbol probability CM::Pr (double matrix) updated
  */
  void demodulate_soft_symbols_relay(cvec c_received_signals, cvec channel_gains, double _2sigma2, 
				     vec beta, vec var_n, vec pow_s, vec mu_n);
  
  /*! 
    \brief CM demodulator for flat Rayleigh fading channel in cooperative scheme: perfect relay
    
    \param c_received_signals received symbol (complex vector)
    \param channel_gains      channel gains (complex vector)
    \param _2sigma2           2*sigma^2
    
    \note channel symbol probability CM::Pr (double matrix) updated
  */
  void demodulate_soft_symbols_relay(cvec c_received_signals, cvec channel_gains, double N0,
				     cvec c_received_signals_rd, cvec channel_gains_rd, double N0_rd);
  
  /*! 
    \brief CM demodulator for flat Rayleigh fading channel in cooperative scheme: soft relay
    
    \param c_received_signals received symbol (complex vector)
    \param channel_gains      channel gains (complex vector)
    \param _2sigma2           2*sigma^2
    
    \note channel symbol probability CM::Pr (double matrix) updated
  */
  void demodulate_soft_symbols_relay(cvec c_received_signals, cvec channel_gains, double N0, 
				     cvec c_received_signals_rd, cvec channel_gains_rd, double N0_rd,
				     vec beta, vec var_n, vec pow_s, vec mu_n);
  
 /*! 
    \brief CM demodulator for flat Rayleigh fading channel in cooperative scheme: Amplify And Forward (AAF)
    
    \param c_received_signals received symbol (complex vector)
    \param channel_gains      channel gains (complex vector)
    \param _2sigma2           2*sigma^2
    
    \note channel symbol probability CM::Pr (double matrix) updated
  */
  void demodulate_soft_symbols_relay_aaf(cvec c_received_signals, cvec channel_gains, double N0, 
				     cvec c_received_signals_rd, cvec channel_gains_rd, double N0_rd,
				     cvec channel_gains_sr, vec factor);
  

  /*! 
    \brief decode symbol based on symbol's a-poteriori probability
    
    Based on data symbol's a-poteriori probability, symbol block
    length and number of possible different input symbols, the most
    likely input data symbol sequence is given.

    \param Apo      data symbol's a-poteriori probability (\a Apo of \a MAP_dec or \a Apo_dataword \a SISO_dec)
    \param symbols  the most likely data symbol sequence
    \param N        symbol block length (\a no_of_symbols)
    \param M        number of possible different input data symbols (\a 2^k)
  */
  void decode_symbol(mat Apo, ivec &symbols, int N, int M);
  
  /*! 
    \brief print parameters to a file pointer.

    printout the parameters of coder, modulator & interleaver.
  */
  void print_parameters(FILE *fp);  

  /*! 
    \brief print parameters to a file stream.

    printout the parameters of coder, modulator & interleaver.
  */
  void print_parameters(fstream &fs);  

  /*! 
    \brief print coding tables to a file pointer.
    
    printout the tables of Codeword NextState & PreviousState.
  */
  void print_coding_tables(FILE *fp);  
  
  /*! 
    \brief print coding tables to a file stream.
    
    printout the tables of Codeword NextState & PreviousState.
  */
  void print_coding_tables(fstream &fs);  
  
  //! return the result filename.
  char* get_result_filename(){return result_filename;}

  //! return the output coded symbol vector length.
  int get_sym_length(){return no_of_symbols;}
  //! return the input data symbol vector length
  int get_info_sym_length(){return no_of_info_symbols;}
  //! return the output coded bit vector length
  int get_coded_bit_length(){return no_of_coded_bits;}
  //! return the input data bit vector length
  int get_info_bit_length(){return no_of_info_bits;}
  //! return the input data bit vector length
  int get_termination_bit_length(){return no_of_tail_bits;}
  //! return the number of turbo iterations 
  int get_iterations(){return iterations;}
  
  //! return the coder mode 
  int get_coder_mode(){return mode;}
   
  //! return the channel symbol probability from demodulator.
  mat get_Pr(){return Pr;}
  //! return the In-phase         channel symbol probability from demodulator.
  mat get_PrI(){return PrI;}
  //! return the Quadrature-phase channel symbol probability from demodulator.
  mat get_PrQ(){return PrQ;} 
  //! return the coded symbol a-posteriori probability after decoding 
  mat get_OPr(){return OPr;}

  //! return the coded symbol a-posteriori probability after decoding : normal domain
  mat get_normal_domain_OPr();


  
  //! return the coded bits a-posteriori probability after decoding 
  mat *get_Coded_Bit_OPr(){return siso->Apo_coded_bit;}
  
  //! return the data bits a-posteriori probability after decoding 
  mat *get_Data_Bit_OPr(){return siso->Apo_data_bit;}  
  
  //! return the actual coding rate 
  double get_coderate(){return (double)no_of_info_bits/(double)no_of_coded_bits; }

  //! return the number of information bits per modulated symbol
  int get_info_bps(){return k; }

  //! return the number of bits per modulated symbol
  int get_bps(){return n; }

  //! calculate SNR per databit Eb, given the symbol energy Es: \f$ Eb = Es/(R*BPS) \f$ where R=coderate, BPS=bits per modulated symbol.
  double compute_Eb(double Es){return Es/ ( (double)no_of_info_bits/(double)no_of_coded_bits*(double)modulation->bits_per_symbol); }
  
  //! calculate the aposteriori log likelihood ratio for each data bits after decoding
  vec get_data_llr();

  vec get_data_extr_llr();

  //! get the mapping table of the complex symbol
  cvec get_SymbolMapping(){return modulation->SymbolMapping; }

  //! get the In-phase mapping table of the complex symbol
  vec get_SymbolMappingI(){return modulation->SymbolMappingI; }

  //! get the Quadrature-phase mapping table of the complex symbol
  vec get_SymbolMappingQ(){return modulation->SymbolMappingQ; }

  //!  convertion of Symbol probability in Log domain to LLR
  void SymProb_to_LLR ( int N, int bps, vec &LLR, mat SymPr );
  //! convertion of Log-Likelihood Ratio (LLR) to Symbol probability in Log domain
  void LLR_to_SymProb ( int N, int bps, vec LLR, mat & SymPr );

  //! convertion of Log-Likelihood Ratio (LLR) to Bit probability in Log domain
  void LLR_to_Prob ( int N, int bps, vec LLR, mat *& BitPr );
  //!  convertion of Bit probability in Log domain to LLR
  void Prob_to_LLR ( int N, int bps, vec &LLR, mat* BitPr );
 
 protected:
  mat Pr;      //!< channel symbol probability from demodulator
  mat OPr;     //!< coded symbol a-posteriori probability after decoding 
  mat OPr2;    //!< coded symbol a-posteriori probability after decoding of second decoder (for TTCM) 
  mat PrI;     //!< In-phase         channel symbol probability from demodulator 
  mat PrQ;     //!< Quadrature-phase channel symbol probability from demodulator 
  
  struct modulation_param *modulation;     //!< modulator of CM
  struct cm_interleaver_control *interleaver; //!< interleaver of CM
  
  struct TCM_parameters  *tcm;             //!< TCM of CM
  struct TTCM_parameters *ttcm;            //!< TTCM of CM
  struct MAP_dec *map;                     //!< MAP decoder for TCM and TTCM
  
  struct BICM_parameters  *bicm;           //!< BICM of CM
  struct BICMID_parameters  *bicmid;       //!< BICMID of CM
  struct SISO_dec *siso;                   //!< SISO decoder for BICM and BICMID
  
  //memory allocation:

  //! allocate memory for TCM coder
  void allocate_memory_tcm();
  //! allocate memory for TTCM coder
  void allocate_memory_ttcm();
  //! allocate memory for BICM/BICMID coder
  void allocate_memory_bicm();
  
  //initialisation:

  //! initialise \ref SP based modulator
  void initModulation_SP();
  //! initialise \ref GRAY coded based modulator
  void initModulation_GRAY();  

  //! non-square QAM
  void APSK_32_modulation(); 

  //! initialise TCM/TTCM coder
  void initTCM();
  //! initialise BICM coder (rate 1/2, 2/3 & 3/4): non-binary Convolutional code
  void initCC();
  //! initialise BICM coder (rate 5/6): employ puncturing on rate 1/2 mother code
  void initPunctureCC();
  
  /*! 
    \brief get generator polynomial from a predefined table for TCM/TTCM
    
    Based on \a k and \a L , the appropriate generator polynomials are
    loaded into GenPoly of \a TCM_parameters.
  */
  void get_gen_poly();

  /*! 
    \brief get generator polynomial and puncturing pattern from a predefined table for BICM/BICMID
    
    Based on \a k and \a L , the appropriate generator polynomials are
    loaded into GenPoly of \a BICM_parameters. Punturing pattern on a
    rate-1/2 mother code for yielding a rate-5/6 BICM coder is loaded
    into puncture of \a BICM_parameters.
  */
  void get_CC_poly();

  /*! 
    \brief initialise channel interleaver: symbol interleaver (ON, OFF) or IQ interleaver (IQ_INTLV)

    \param no_of_symbols number of coded symbols, or the interleaver length.
    
    channel interleaver table(s) will be generated and stored in file(s).
  */
  void init_channel_interleaver(int no_of_symbols);
  //! Free Channel Interleaver memory
  void free_channel_interleaver();
  //! initialise random interleaver
  void init_random_interleaver(int block_length, ivec &table, char *txt);

  //! initialise In-phase Quadrature-phase random interleaver
  void init_iq_tables();
  //! get In-phase table, used by init_iq_tables()
  void get_I_table();
  //! get Quadrature-phase table, used by init_iq_tables()
  void get_Q_table();
  
  //! initialise TTCM interleaver: Odd Even Separated (oes) symbol intereaver
  void init_TTCM_interleaver();  
  //! initialise Odd-Even-Separated symbol interleaver for TTCM
  void init_OES_interleaver(int block_length, ivec &i_table);
  //! find termination locations for 2nd encoder of TTCM used by init_OES_interleaver()
  int  intlv_termination_locat(int block_length, int L, ivec &i_table, ivec &last);

  //encoding:
  
  //! TCM encoder
  void TCMEnc( ivec, ivec &);
  //! TTCM encoder
  void TurboTCMEnc( ivec, ivec &);
  //! BICM encoder
  void BICMEnc( ivec, ivec &);
  //! punctured BICM encoder
  void BICMEnc_Puncture( ivec, ivec &);

  //decoding:
  
  //! Jacobian logarithmic summation: based on the \a decoder_type obtions.
  double jacolog(double x, double y);
  
  /*! 
    \brief Logarithmic MAP decoder for TCM
    
    \param N   the block length
    \param M   the number of trellis branches
    \param S   the number of trellis states
    \param Ps  Previous state table
    \param Ns  Next state table
    \param Lb  Codeword (branch label)
    \param Apr a-priori probability of dataword
    \param Ip  Ip[i](j,m) Branch transition metric at time i from state j with input dataword m
    \param Apo a-posteriori probability of dataword
    \param OPr a-posteriori probability of codeword
  */
  void log_mapdec(int N, int M, int S, imat Ps, imat Ns, imat Lb, 
		  mat Apr, mat *Ip, 
		  mat &Apo, mat &OPr);
  
  /*! 
    \brief Logarithmic MAP decoder for TTCM, assumming no Apr information from other source
    
    \param N   the block length
    \param M   the number of trellis branches
    \param S   the number of trellis states
    \param Ps  Previous state table
    \param Ns  Next state table
    \param Lb  Codeword (branch label)
    \param Apr a-priori probability of dataword
    \param Ip1 decoder 1: Ip1[i](j,m) Branch transition metric at time i from state j with input dataword m
    \param Ip2 decoder 2: Ip2[i](j,m) Branch transition metric at time i from state j with input dataword m
    \param Apo a-posteriori probability of dataword
    \param OPr  a-posteriori probability of codeword (actual value after taking into account of OPr from decoder 1 & 2)
    \param OPr2 a-posteriori probability of codeword from decoder 2
  */
  void log_TTCM_dec(int N, int M, int S, imat Ps, imat Ns, imat Lb,
		    mat Apr, mat *Ip1, mat *Ip2,
		    mat &Apo, mat &OPr, mat &OPr2);

    /*! 
    \brief Logarithmic MAP decoder for TTCM, can be used for joint CM and source coding where Apr is provided by the source decoder
    
    \param N   the block length
    \param M   the number of trellis branches
    \param S   the number of trellis states
    \param Ps  Previous state table
    \param Ns  Next state table
    \param Lb  Codeword (branch label)
    \param Apr a-priori probability of dataword provided by the source decoder or any other source
    \param Ip1 decoder 1: Ip1[i](j,m) Branch transition metric at time i from state j with input dataword m
    \param Ip2 decoder 2: Ip2[i](j,m) Branch transition metric at time i from state j with input dataword m
    \param Apo a-posteriori probability of dataword
    \param OPr  a-posteriori probability of codeword (actual value after taking into account of OPr from decoder 1 & 2)
    \param OPr2 a-posteriori probability of codeword from decoder 2
  */
  void log_TTCM_dec_apr(int N, int M, int S, imat Ps, imat Ns, imat Lb,
			mat Apr, mat *Ip1, mat *Ip2,
			mat &Apo, mat &OPr, mat &OPr2);
  
  /*! 
    \brief Logarithmic SISO(MAP) decoder for BICM
    
    \param N             the block length
    \param lD            the number of databits per codeword
    \param D             the number of trellis branches
    \param S             the number of trellis states
    \param Ps            Previous state table
    \param Ns            Next state table
    \param Lb            Codeword (branch label)
    \param Apr_coded_bit a-priori     probability of codedbit
    \param Apo_coded_bit a-posteriori probability of codedbit
    \param Apo_data_bit  a-posteriori probability of databit
    \param Apo_dataword  a-posteriori probability of dataword
    \param Terminated    Code termination \a Terminated
    \param frame_index   if channel interleaver is \e J times longer than the coded block, \f$ frame\_index \in\{0\ldots J-1\} \f$
  */
  void SISO_dec_log(int N, int lD, int D, int S, imat Ps, imat Ns, imat Lb, 
		    mat *Apr_coded_bit, mat *&Apo_coded_bit, mat *&Apo_data_bit, 
		    mat &Apo_dataword, int Terminated, int frame_index);

  /*! 
    \brief Logarithmic SISO(MAP) decoder for any coding scheme
    
    \param N             the block length
    \param lD            the number of databits per codeword
    \param D             the number of trellis branches
    \param S             the number of trellis states
    \param Ps            Previous state table
    \param Ns            Next state table
    \param Lb            Codeword (branch label)
    \param Apr_codeword  a-priori  probability of codeword
    \param Apr_dataword  a-priori  probability of dataword
    \param Apo_codedword a-posteriori probability of codeword
    \param Apo_dataword  a-posteriori probability of dataword
    \param Terminated    Code termination \a Terminated
    \param frame_index   if channel interleaver is \e J times longer than the coded block, \f$ frame\_index \in\{0\ldots J-1\} \f$
  */
  void SISO_dec_log(int N, int lD, int D, int S, imat Ps, imat Ns, imat Lb, 
		    mat Apr_codeword, mat Apr_dataword,mat &Apo_codeword, mat &Apo_dataword, 
		    int Terminated, int frame_index);
  
  /*! 
    \brief Logarithmic SISO(MAP) decoder for BICM/BICMID
    
    \param N             the block length
    \param lD            the number of databits per codeword
    \param D             the number of trellis branches
    \param S             the number of trellis states
    \param Ps            Previous state table
    \param Ns            Next state table
    \param Lb            Codeword (branch label)
    \param Apr_coded_bit a-priori     probability of codedbit
    \param Apr_data_bit  a-priori     probability of databit
    \param Apo_coded_bit a-posteriori probability of codedbit
    \param Apo_data_bit  a-posteriori probability of databit
    \param Apo_dataword  a-posteriori probability of dataword
    \param Terminated    Code termination \a Terminated
  */
  void BICM_ID_dec_log(int N, int lD, int D, int S, imat Ps, imat Ns, imat Lb, mat *Apr_coded_bit, mat *Apr_data_bit,
		       mat *&Apo_coded_bit, mat *&Apo_data_bit, mat &Apo_dataword, int Terminated);

  /*! 
    \brief Logarithmic SISO(MAP) decoder for BICM/BICMID employing puncturing
    
    \param N             the block length
    \param lD            the number of databits per codeword
    \param D             the number of trellis branches
    \param S             the number of trellis states
    \param Ps            Previous state table
    \param Ns            Next state table
    \param Lb            Codeword (branch label)
    \param Apr_coded_bit a-priori     probability of codedbit
    \param Apr_data_bit  a-priori     probability of databit
    \param Apo_coded_bit a-posteriori probability of codedbit
    \param Apo_data_bit  a-posteriori probability of databit
    \param Apo_dataword  a-posteriori probability of dataword
    \param Terminated    Code termination \a Terminated
  */
  void BICM_ID_dec_log_pun(int N, int lD, int D, int S, imat Ps, imat Ns, imat Lb, mat *Apr_coded_bit, mat *Apr_data_bit,
			   mat *&Apo_coded_bit, mat *&Apo_data_bit, mat &Apo_dataword, int Terminated);

  /*! 
    \brief demodulator of BICMID which outputs the extrinsic probability of coded bits
    
    \param N              the block length
    \param lC             the number of codedbits per modulated symbol
    \param C              the number of modulation level
    \param Apo_coded_bit  a-posteriori probability of codedbit
    \param Pr             original channel symbol probability from demodulator
    \param Extr_coded_bit extrinsic probability of codedbit
    \param frame_index    if channel interleaver is \e J times longer than the coded block, \f$ frame\_index \in\{0\ldots J-1\} \f$
  */
  void extrinsic_demod_log(int N, int lC, int C, mat *Apo_coded_bit, mat Pr, mat *&Extr_coded_bit, int frame_index);
  
  /*! 
    \brief depuncturer for punctured BICM code
  
    \param Apr_coded_bit    (input)  a-priori probability of codedbit from demodulator
    \param p_Apr_coded_bit  (output) a-priori probability of codedbit after depunturing
    \param N                the block length
    \param dataword_length  the number of databit per coded symbol
    \param codeword_length  the number of codedbit per coded symbol
    \param puncture         punturing pattern
    \param p_lC             mother code's codedbit per coded symbol
    \param frame_index   if channel interleaver is \e J times longer than the coded block, \f$ frame\_index \in\{0\ldots J-1\} \f$
  */
  void depuncture(mat *Apr_coded_bit, mat *&p_Apr_coded_bit, int N, int dataword_length, int codeword_length,
		  ivec puncture, int p_lC, int frame_index);
  
  /*! 
    \brief puncturer for punctured BICM code
  
    \param Apo_coded_bit    (output  extrinsic probability of codedbit after puncturing
    \param p_Apo_coded_bit  (input)  extrinsic probability of codedbit from decoder
    \param N                the block length
    \param dataword_length  the number of databit per coded symbol
    \param codeword_length  the number of codedbit per coded symbol
    \param puncture         punturing pattern
    \param p_lC             mother code's codedbit per coded symbol
    \param frame_index   if channel interleaver is \e J times longer than the coded block, \f$ frame\_index \in\{0\ldots J-1\} \f$
  */
  void puncture(mat *&Apo_coded_bit, mat *p_Apo_coded_bit, int N, int dataword_length, int codeword_length,
		ivec puncture, int p_lC, int frame_index);

  /*! 
    \brief calculate dataword's and databit's a-posteriori probability from coded bit's a-posteriori probability of decoder
    
    \param Apo_dataword     (output) dataword's a-posteriori probability
    \param Apo_data_bit     (output) databit's  a-posteriori probability
    \param p_Apo_data_bit   (input)  databit's  a-posteriori probability from decoder
    \param N                the block length
    \param dataword_length  the number of databit per coded symbol
    \param p_lD             mother code's databit per coded symbol
  */
  void get_Apo_dataword(mat &Apo_dataword, mat *&Apo_data_bit, mat *p_Apo_data_bit, int N, int dataword_length, int p_lD);
  
  /*! 
    \brief calculate the databit's a-priori probability for punctured BICM decoding
    
    \param p_Apr_data_bit   (input)  databit's  a-priori probability 
    \param Apr_data_bit     (output) databit's  a-priori probability
    \param N                the block length
    \param dataword_length  the number of databit per coded symbol
    \param p_lD             mother code's databit per coded symbol
  */
  void get_p_Apr_data_bit(mat *&p_Apr_data_bit, mat *Apr_data_bit, int N, int dataword_length, int p_lD);
  

  //Interleaving:

  //! interleaving integer vector of length block_length based on i_table 
  void intlvi( ivec x, ivec &y, int block_length, ivec i_table);
  //! deinterleaving integer vector of length block_length based on i_table 
  void de_intlvi( ivec x, ivec &y, int block_length, ivec i_table);  
  
  //! interleaving double matrix (2-dimension) of length block_length based on i_table 
  void intlvd_2d( mat x, mat &y, int block_length, ivec i_table);
  //! deinterleaving double matrix (2-dimension) of length block_length based on i_table 
  void de_intlvd_2d( mat x, mat &y, int block_length, ivec i_table);
  
  //! interleaving double *matrix (3-dimension) of length block_length based on i_table 
  void intlvm_3d( mat *x, mat *&y, int block_length, ivec i_table);
  //! deinterleaving double *matrix (3-dimension) of length block_length based on i_table 
  void de_intlvm_3d (mat *x, mat *&y, int block_length, ivec i_table);
  
  //! interleaving index-th bit's integer vector (row of imat) of length block_length based on i_table 
  void bit_intlvi( imat x, imat &y, int block_length, ivec i_table, int index);
  //! deinterleaving index-th bit's integer vector (row of imat) of length block_length based on i_table 
  void bit_de_intlvi( imat x, imat &y, int block_length, ivec i_table, int index);
  
  //! interleaving index-th bit's double *matrix (3-dimension) of length block_length based on i_table 
  void bit_intlvm_3d (mat * x, mat *&y, int block_length, int m, ivec i_table, int index);
  //! deinterleaving index-th bit's double *matrix (3-dimension) of length block_length based on i_table 
  void bit_de_intlvm_3d (mat * x, mat *&y, int block_length, int m, ivec i_table, int index);

  //! interleaving In-phase and Quadrature-phase of a complex vector
  void iq_intlvc( cvec x, cvec &y, int block_length, ivec i_table, ivec q_table);
  //! deinterleaving In-phase and Quadrature-phase of a complex vector
  void iq_de_intlvc( cvec x, cvec &y, int block_length, ivec i_table, ivec q_table);


  //demodulation:
  
  //! IQ demodulator for AWGN channel based on complex received_signals and 2*sigma^2. \a PrI, \a PrQ and \a Pr are updated 
  void demodulate_soft_symbols_IQ(cvec c_received_signals, double _2sigma2);
  //! IQ demodulator for flat Rayleigh fading channel based on complex received_signals and 2*sigma^2. \a PrI, \a PrQ and \a Pr are updated 
  void demodulate_soft_symbols_IQ(cvec c_received_signals, cvec channel_gains, double _2sigma2);
  //! computing \a Pr from \a PrI and \a PrQ
  void compute_Pr_from_PrIQ(mat &Pr, mat PrI, mat PrQ);

 private:
  char result_filename[500];  //!< name of result file
  int mode;               //!< CM mode: \ref TCM, \ref TTCM, \ref BICM, \ref BICMID
  int modulation_type;    //!< modulation type:  \ref CPSK, \ref CQAM 
  int interleaver_mode;   //!< interleaver mode: \ref OFF, \ref ON, \ref IQ_INTLV  
  int Terminated;         //!< code termination: \ref OFF, \ref ON
  int decoder_type;       //!< decoding type: \ref CM_EXACT_LOG_MAP, \ref CM_APPROX_LOG_MAP, \ref CM_MAX_LOG_MAP

  int k;                  //!< length of dataword
  int n;                  //!< length of codeword
  int L;                  //!< code memory length
  int iterations;         //!< number of turbo iterations 
  
  int max_no_of_symbols;  //!< maximum number of coded symbols in a block, for variable length CM

  int no_of_symbols;      //!< number of coded symbols in a block 
  int no_of_info_symbols; //!< number of information (input data) symbols in one block 
  int no_of_coded_bits;   //!< number of output coded bits in one block
  int no_of_info_bits;    //!< number of information (input data) bits in one block 
  int no_of_tail_bits;    //!< number of termination (tail) bits in one block 
  int no_of_input_bits;   //!< number of input bits (\a no_of_info_bits + \a no_of_tail_bits ) in one block 

  //! Log-domain: compute bit probability BitPr from symbol probability Pr; N=total symbol, lM=bit per modulated symbol
  void Pr_to_BitPr_log(int N, int lM, mat Pr, mat *&BitPr, int frame_index);
  //! Log-domain: compute symbol probability Pr from bit probability BitPr; N=total symbol, lM=bit per modulated symbol
  void BitPr_to_Pr_log(int N, int lM, mat &Pr, mat *BitPr, int frame_index);

  //! compute bit matrix (bits_block) from symbol vector (symbol); word_length=col block_length=row of bits_block
  void symbol_to_bits(int word_length, int block_length, ivec symbols, imat &bits_block);
  //! compute symbol vector (symbol) from bit matrix (bits_block); word_length=col block_length=row of bits_block
  void bits_to_symbol(int word_length, int block_length, imat bits_block, ivec &symbols);
  //! compute bit vector of length=block_length*word_length from symbol vector
  ivec symbol_to_bits_seq(int word_length, int block_length, ivec symbols);
  //! compute symbol vector from bit vector of length=block_length*word_length
  ivec bits_seq_to_symbol(int word_length, int block_length, ivec bits_seq);


  //! printout error message and exit
  void s_error ( const char * message );
  //! Set Partitioning labelling for QAM
  void makebits(int s, int N, imat &bit1, imat &bit2);
  //! correct rounding error to the precision of 6 decimal places
  double correct_rounding_error(double distance);
  //! compute decimal number from octal number
  int octal_to_decimal(int gen);
  
};

//! Jacobian logarithmic summation: \a decoder_type= \ref CM_EXACT_LOG_MAP
double jacolog_1( double x, double y);
//! Jacobian logarithmic summation: \a decoder_type= \ref CM_APPROX_LOG_MAP
double jacolog_2( double x, double y);
//! Jacobian logarithmic summation: \a decoder_type= \ref CM_MAX_LOG_MAP
double jacolog_3( double x, double y);


#endif
