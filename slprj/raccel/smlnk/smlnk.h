#ifndef smlnk_h_
#define smlnk_h_
#ifndef smlnk_COMMON_INCLUDES_
#define smlnk_COMMON_INCLUDES_
#include <stdlib.h>
#include "sl_AsyncioQueue/AsyncioQueueCAPI.h"
#include "rtwtypes.h"
#include "sigstream_rtw.h"
#include "simtarget/slSimTgtSigstreamRTW.h"
#include "simtarget/slSimTgtSlioCoreRTW.h"
#include "simtarget/slSimTgtSlioClientsRTW.h"
#include "simtarget/slSimTgtSlioSdiRTW.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "raccel.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "rt_logging_simtarget.h"
#include "rt_nonfinite.h"
#include "math.h"
#include "dt_info.h"
#include "ext_work.h"
#endif
#include "smlnk_types.h"
#include <stddef.h>
#include "rtw_modelmap_simtarget.h"
#include "rt_defines.h"
#include <string.h>
#define MODEL_NAME smlnk
#define NSAMPLE_TIMES (3) 
#define NINPUTS (0)       
#define NOUTPUTS (0)     
#define NBLOCKIO (14) 
#define NUM_ZC_EVENTS (0) 
#ifndef NCSTATES
#define NCSTATES (12)   
#elif NCSTATES != 12
#error Invalid specification of NCSTATES defined in compiler command
#endif
#ifndef rtmGetDataMapInfo
#define rtmGetDataMapInfo(rtm) (*rt_dataMapInfoPtr)
#endif
#ifndef rtmSetDataMapInfo
#define rtmSetDataMapInfo(rtm, val) (rt_dataMapInfoPtr = &val)
#endif
#ifndef IN_RACCEL_MAIN
#endif
typedef struct { real_T krtg4hkewq [ 6 ] ; real_T hx4zbpyorm [ 6 ] ; real_T
hfcc51pulu ; real_T nwsu2llks5 ; real_T aitunyvbo3 [ 9 ] ; real_T a3uqevckku
[ 2 ] ; real_T mdqy0sybu0 [ 4 ] ; real_T jxcagdmfiz [ 3 ] ; real_T mg3dqlxqgc
[ 3 ] ; real_T bkbzioulom [ 24 ] ; real_T ikfxwn30vz [ 6 ] ; real_T
cu04fobhdc ; real_T h04naoubyp ; real_T dzp0ufmz11 ; } B ; typedef struct {
real_T pydj2pulvz ; real_T h4kbtduxza ; real_T jacddttya0 ; real_T jsubjbn3x3
; real_T o0rlsmhvnx ; real_T mj3ycsnbqz ; real_T jlqd5uv2t5 ; real_T
bw0m2qixpq ; real_T bgxyound4l ; real_T jlfkntbhq3 ; real_T isxzobxtuk ;
real_T ki1qusscch ; real_T aswygtmbgv ; struct { void * LoggedData [ 3 ] ; }
kvnsin1yg3 ; struct { void * LoggedData ; } l25hiwzvr2 ; struct { void *
LoggedData [ 3 ] ; } eghn2cm5gy ; struct { void * LoggedData [ 3 ] ; }
guy2kna2li ; struct { void * LoggedData ; } gsh54m1bmb ; struct { void *
LoggedData [ 2 ] ; } jvhux0m2vh ; struct { void * LoggedData [ 2 ] ; }
crk1vj15iv ; struct { void * AQHandles ; } fsz1bpdsia ; struct { void *
LoggedData ; } piepfykkqk ; struct { void * LoggedData ; } c3ebnl0is0 ;
int32_T e3y0cazldt ; int32_T ejujdun315 ; int32_T f2x2yaoqgy ; int32_T
d34l4kjipg ; int32_T f3nbwloi5j ; boolean_T njpxizyule ; boolean_T cjhm3acixb
; boolean_T lxythyf1cb ; boolean_T nh4tvjogpv ; boolean_T ba1rd1ajhl ;
boolean_T flkg1gk54m ; boolean_T orxmg0nmrg ; boolean_T lbm0o1qa52 ;
boolean_T av31m02ta4 ; boolean_T briiwtgmsq ; boolean_T p104kbp3ah ;
boolean_T fdovcjpqrp ; boolean_T itne4ou4c4 ; boolean_T ffn3ukflan ;
boolean_T a0uid4l0f4 ; boolean_T i5lo0j2nbo ; boolean_T jwsyzsowec ;
boolean_T lnsdae4spp ; } DW ; typedef struct { real_T cm1oq22mti [ 6 ] ;
real_T ffddlbb5mp [ 6 ] ; } X ; typedef struct { real_T cm1oq22mti [ 6 ] ;
real_T ffddlbb5mp [ 6 ] ; } XDot ; typedef struct { boolean_T cm1oq22mti [ 6
] ; boolean_T ffddlbb5mp [ 6 ] ; } XDis ; typedef struct {
rtwCAPI_ModelMappingInfo mmi ; } DataMapInfo ; struct P_ { real_T Sample_Time
; real_T _IC [ 6 ] ; real_T _IC_mctvldpkh3 [ 6 ] ; } ; extern const char_T *
RT_MEMORY_ALLOCATION_ERROR ; extern B rtB ; extern X rtX ; extern DW rtDW ;
extern P rtP ; extern mxArray * mr_smlnk_GetDWork ( ) ; extern void
mr_smlnk_SetDWork ( const mxArray * ssDW ) ; extern mxArray *
mr_smlnk_GetSimStateDisallowedBlocks ( ) ; extern const
rtwCAPI_ModelMappingStaticInfo * smlnk_GetCAPIStaticMap ( void ) ; extern
SimStruct * const rtS ; extern DataMapInfo * rt_dataMapInfoPtr ; extern
rtwCAPI_ModelMappingInfo * rt_modelMapInfoPtr ; void MdlOutputs ( int_T tid )
; void MdlOutputsParameterSampleTime ( int_T tid ) ; void MdlUpdate ( int_T
tid ) ; void MdlTerminate ( void ) ; void MdlInitializeSizes ( void ) ; void
MdlInitializeSampleTimes ( void ) ; SimStruct * raccel_register_model (
ssExecutionInfo * executionInfo ) ;
#endif
