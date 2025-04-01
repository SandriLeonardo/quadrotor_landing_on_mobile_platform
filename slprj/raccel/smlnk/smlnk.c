#include "smlnk.h"
#include "rtwtypes.h"
#include "mwmathutil.h"
#include "smlnk_private.h"
#include "rt_logging_mmi.h"
#include "smlnk_capi.h"
#include "smlnk_dt.h"
extern void * CreateDiagnosticAsVoidPtr_wrapper ( const char * id , int nargs
, ... ) ; extern ssExecutionInfo gblExecutionInfo ; RTWExtModeInfo *
gblRTWExtModeInfo = NULL ; void raccelForceExtModeShutdown ( boolean_T
extModeStartPktReceived ) { if ( ! extModeStartPktReceived ) { boolean_T
stopRequested = false ; rtExtModeWaitForStartPkt ( gblRTWExtModeInfo , 2 , &
stopRequested ) ; } rtExtModeShutdown ( 2 ) ; }
#include "slsv_diagnostic_codegen_c_api.h"
#include "slsa_engine_exec.h"
#ifdef RSIM_WITH_SOLVER_MULTITASKING
boolean_T gbl_raccel_isMultitasking = 1 ;
#else
boolean_T gbl_raccel_isMultitasking = 0 ;
#endif
boolean_T gbl_raccel_tid01eq = 1 ; int_T gbl_raccel_NumST = 3 ; const char_T
* gbl_raccel_Version = "24.1 (R2024a) 19-Nov-2023" ; void
raccel_setup_MMIStateLog ( SimStruct * S ) {
#ifdef UseMMIDataLogging
rt_FillStateSigInfoFromMMI ( ssGetRTWLogInfo ( S ) , & ssGetErrorStatus ( S )
) ;
#else
UNUSED_PARAMETER ( S ) ;
#endif
} static DataMapInfo rt_dataMapInfo ; DataMapInfo * rt_dataMapInfoPtr = &
rt_dataMapInfo ; rtwCAPI_ModelMappingInfo * rt_modelMapInfoPtr = & (
rt_dataMapInfo . mmi ) ; int_T enableFcnCallFlag [ ] = { 1 , 1 , 1 } ; const
char * raccelLoadInputsAndAperiodicHitTimes ( SimStruct * S , const char *
inportFileName , int * matFileFormat ) { return rt_RAccelReadInportsMatFile (
S , inportFileName , matFileFormat ) ; }
#include "simstruc.h"
#include "fixedpoint.h"
#include "slsa_engine_exec.h"
#include "simtarget/slSimTgtSLExecSimBridge.h"
#define mjvsty0x4r (-1)
B rtB ; X rtX ; DW rtDW ; static SimStruct model_S ; SimStruct * const rtS =
& model_S ; void MdlInitialize ( void ) { int32_T i ; for ( i = 0 ; i < 6 ; i
++ ) { rtX . cm1oq22mti [ i ] = rtP . _IC [ i ] ; rtX . ffddlbb5mp [ i ] =
rtP . _IC_mctvldpkh3 [ i ] ; } rtDW . njpxizyule = false ; rtDW . e3y0cazldt
= mjvsty0x4r ; rtDW . itne4ou4c4 = false ; rtDW . f2x2yaoqgy = mjvsty0x4r ;
rtDW . flkg1gk54m = false ; rtDW . orxmg0nmrg = false ; rtDW . lbm0o1qa52 =
false ; rtDW . av31m02ta4 = false ; rtDW . briiwtgmsq = false ; rtDW .
p104kbp3ah = false ; rtDW . fdovcjpqrp = false ; rtDW . pydj2pulvz = 0.0 ;
rtDW . lxythyf1cb = true ; rtDW . h4kbtduxza = 0.0 ; rtDW . nh4tvjogpv = true
; rtDW . jacddttya0 = 0.0 ; rtDW . ba1rd1ajhl = true ; rtDW . mj3ycsnbqz =
0.0 ; rtDW . jlqd5uv2t5 = 0.0 ; rtDW . cjhm3acixb = false ; rtDW . ejujdun315
= mjvsty0x4r ; rtDW . isxzobxtuk = 0.0 ; rtDW . i5lo0j2nbo = true ; rtDW .
ki1qusscch = 0.0 ; rtDW . jwsyzsowec = true ; rtDW . aswygtmbgv = 0.0 ; rtDW
. lnsdae4spp = true ; rtDW . a0uid4l0f4 = false ; rtDW . f3nbwloi5j =
mjvsty0x4r ; rtDW . ffn3ukflan = false ; rtDW . d34l4kjipg = mjvsty0x4r ; }
void MdlStart ( void ) { { bool externalInputIsInDatasetFormat = false ; void
* pISigstreamManager = rt_GetISigstreamManager ( rtS ) ;
rtwISigstreamManagerGetInputIsInDatasetFormat ( pISigstreamManager , &
externalInputIsInDatasetFormat ) ; if ( externalInputIsInDatasetFormat ) { }
} { { { bool isStreamoutAlreadyRegistered = false ; { sdiSignalSourceInfoU
srcInfo ; sdiLabelU loggedName = sdiGetLabelFromChars ( "" ) ; sdiLabelU
origSigName = sdiGetLabelFromChars ( "" ) ; sdiLabelU propName =
sdiGetLabelFromChars ( "" ) ; sdiLabelU blockPath = sdiGetLabelFromChars (
"smlnk/Demux1" ) ; sdiLabelU blockSID = sdiGetLabelFromChars ( "" ) ;
sdiLabelU subPath = sdiGetLabelFromChars ( "" ) ; sdiDims sigDims ; sdiLabelU
sigName = sdiGetLabelFromChars ( "" ) ; sdiAsyncRepoDataTypeHandle hDT =
sdiAsyncRepoGetBuiltInDataTypeHandle ( DATA_TYPE_DOUBLE ) ; { sdiComplexity
sigComplexity = REAL ; sdiSampleTimeContinuity stCont =
SAMPLE_TIME_CONTINUOUS ; int_T sigDimsArray [ 2 ] = { 1 , 1 } ; sigDims .
nDims = 2 ; sigDims . dimensions = sigDimsArray ; srcInfo . numBlockPathElems
= 1 ; srcInfo . fullBlockPath = ( sdiFullBlkPathU ) & blockPath ; srcInfo .
SID = ( sdiSignalIDU ) & blockSID ; srcInfo . subPath = subPath ; srcInfo .
portIndex = 2 + 1 ; srcInfo . signalName = sigName ; srcInfo . sigSourceUUID
= 0 ; rtDW . fsz1bpdsia . AQHandles = sdiStartAsyncioQueueCreation ( hDT , &
srcInfo , rt_dataMapInfo . mmi . InstanceMap . fullPath ,
"0babbd24-596c-4f0a-b96e-f723f25b1684" , sigComplexity , & sigDims ,
DIMENSIONS_MODE_FIXED , stCont , "" ) ; sdiCompleteAsyncioQueueCreation (
rtDW . fsz1bpdsia . AQHandles , hDT , & srcInfo ) ; if ( rtDW . fsz1bpdsia .
AQHandles ) { sdiSetSignalSampleTimeString ( rtDW . fsz1bpdsia . AQHandles ,
"0.0001" , 0.0001 , ssGetTFinal ( rtS ) ) ; sdiSetSignalRefRate ( rtDW .
fsz1bpdsia . AQHandles , 0.0 ) ; sdiSetRunStartTime ( rtDW . fsz1bpdsia .
AQHandles , ssGetTaskTime ( rtS , 1 ) ) ; sdiAsyncRepoSetSignalExportSettings
( rtDW . fsz1bpdsia . AQHandles , 1 , 0 ) ; sdiAsyncRepoSetSignalExportName (
rtDW . fsz1bpdsia . AQHandles , loggedName , origSigName , propName ) ; }
sdiFreeLabel ( sigName ) ; sdiFreeLabel ( loggedName ) ; sdiFreeLabel (
origSigName ) ; sdiFreeLabel ( propName ) ; sdiFreeLabel ( blockPath ) ;
sdiFreeLabel ( blockSID ) ; sdiFreeLabel ( subPath ) ; } } if ( !
isStreamoutAlreadyRegistered ) { } } } } MdlInitialize ( ) ; } void
MdlOutputs ( int_T tid ) { real_T lkgf4mnmai [ 12 ] ; real_T T ; real_T T_y ;
real_T alpha ; real_T ex ; real_T ey ; real_T ez ; real_T max_lateral_accel ;
real_T phi_d ; real_T phi_dot_d ; real_T psi_dot_d ; real_T theta_dot_d ;
real_T x ; int32_T i ; for ( i = 0 ; i < 6 ; i ++ ) { rtB . krtg4hkewq [ i ]
= rtX . cm1oq22mti [ i ] ; rtB . hx4zbpyorm [ i ] = rtX . ffddlbb5mp [ i ] ;
} rtB . hfcc51pulu = ssGetT ( rtS ) ; if ( ssIsSampleHit ( rtS , 1 , 0 ) ) {
rtDW . e3y0cazldt = mjvsty0x4r ; if ( rtB . hfcc51pulu < 0.0 ) { rtB .
aitunyvbo3 [ 0 ] = 0.0 ; rtB . aitunyvbo3 [ 3 ] = 0.0 ; rtB . aitunyvbo3 [ 1
] = 0.0 ; rtB . aitunyvbo3 [ 4 ] = 0.0 ; rtB . aitunyvbo3 [ 2 ] = 0.0 ; rtB .
aitunyvbo3 [ 5 ] = 0.0 ; } else if ( rtB . hfcc51pulu <= 10.0 ) { rtB .
aitunyvbo3 [ 0 ] = 0.30000000000000004 * rtB . hfcc51pulu ; rtB . aitunyvbo3
[ 3 ] = 0.30000000000000004 ; rtB . aitunyvbo3 [ 1 ] = 0.50000000000000011 *
rtB . hfcc51pulu ; rtB . aitunyvbo3 [ 4 ] = 0.50000000000000011 ; rtB .
aitunyvbo3 [ 2 ] = 0.79999999999999993 * rtB . hfcc51pulu ; rtB . aitunyvbo3
[ 5 ] = 0.79999999999999993 ; } else { rtB . aitunyvbo3 [ 0 ] = 3.0 ; rtB .
aitunyvbo3 [ 3 ] = 0.0 ; rtB . aitunyvbo3 [ 1 ] = 5.0 ; rtB . aitunyvbo3 [ 4
] = 0.0 ; rtB . aitunyvbo3 [ 2 ] = 8.0 ; rtB . aitunyvbo3 [ 5 ] = 0.0 ; }
rtDW . f2x2yaoqgy = mjvsty0x4r ; rtB . bkbzioulom [ 0 ] = 1.0 ; rtB .
bkbzioulom [ 1 ] = 9.81 ; rtB . bkbzioulom [ 2 ] = 0.1 ; rtB . bkbzioulom [ 3
] = 0.1 ; rtB . bkbzioulom [ 4 ] = 0.2 ; rtB . aitunyvbo3 [ 6 ] = 0.0 ; rtB .
bkbzioulom [ 5 ] = 20.0 ; rtB . bkbzioulom [ 8 ] = 0.001 ; rtB . bkbzioulom [
11 ] = 0.18 ; rtB . bkbzioulom [ 14 ] = 0.01 ; rtB . bkbzioulom [ 17 ] = 0.01
; rtB . bkbzioulom [ 20 ] = 0.01 ; rtB . aitunyvbo3 [ 7 ] = 0.0 ; rtB .
bkbzioulom [ 6 ] = 20.0 ; rtB . bkbzioulom [ 9 ] = 0.001 ; rtB . bkbzioulom [
12 ] = 0.18 ; rtB . bkbzioulom [ 15 ] = 0.01 ; rtB . bkbzioulom [ 18 ] = 0.01
; rtB . bkbzioulom [ 21 ] = 0.01 ; rtB . aitunyvbo3 [ 8 ] = 0.0 ; rtB .
bkbzioulom [ 7 ] = 49.8 ; rtB . bkbzioulom [ 10 ] = 0.2 ; rtB . bkbzioulom [
13 ] = 0.1 ; rtB . bkbzioulom [ 16 ] = 0.01 ; rtB . bkbzioulom [ 19 ] = 0.01
; rtB . bkbzioulom [ 22 ] = 0.01 ; rtB . bkbzioulom [ 23 ] = rtP .
Sample_Time ; for ( i = 0 ; i < 6 ; i ++ ) { lkgf4mnmai [ i ] = rtB .
krtg4hkewq [ i ] ; lkgf4mnmai [ i + 6 ] = rtB . hx4zbpyorm [ i ] ; } rtDW .
ejujdun315 = mjvsty0x4r ; ez = rtB . aitunyvbo3 [ 2 ] - lkgf4mnmai [ 2 ] ;
rtDW . pydj2pulvz += ez * rtB . bkbzioulom [ 23 ] ; ez = - ( ( ( ( rtB .
aitunyvbo3 [ 5 ] - lkgf4mnmai [ 5 ] ) * rtB . bkbzioulom [ 10 ] + rtB .
bkbzioulom [ 7 ] * ez ) + rtB . bkbzioulom [ 13 ] * rtDW . pydj2pulvz ) + rtB
. aitunyvbo3 [ 8 ] ) + rtB . bkbzioulom [ 1 ] ; T = rtB . bkbzioulom [ 0 ] /
( muDoubleScalarCos ( lkgf4mnmai [ 6 ] ) * muDoubleScalarCos ( lkgf4mnmai [ 7
] ) ) * ( rtB . bkbzioulom [ 1 ] + ez ) ; ex = rtB . aitunyvbo3 [ 0 ] -
lkgf4mnmai [ 0 ] ; rtDW . h4kbtduxza += ex * rtB . bkbzioulom [ 23 ] ; ey =
rtB . aitunyvbo3 [ 1 ] - lkgf4mnmai [ 1 ] ; rtDW . jacddttya0 += ey * rtB .
bkbzioulom [ 23 ] ; max_lateral_accel = rtB . bkbzioulom [ 1 ] *
0.42279321873816178 ; ex = muDoubleScalarMax ( muDoubleScalarMin ( ( ( ( rtB
. aitunyvbo3 [ 3 ] - lkgf4mnmai [ 3 ] ) * rtB . bkbzioulom [ 8 ] + rtB .
bkbzioulom [ 5 ] * ex ) + rtB . bkbzioulom [ 11 ] * rtDW . h4kbtduxza ) + rtB
. aitunyvbo3 [ 6 ] , max_lateral_accel ) , - max_lateral_accel ) ; ey =
muDoubleScalarMax ( muDoubleScalarMin ( ( ( ( rtB . aitunyvbo3 [ 4 ] -
lkgf4mnmai [ 4 ] ) * rtB . bkbzioulom [ 9 ] + rtB . bkbzioulom [ 6 ] * ey ) +
rtB . bkbzioulom [ 12 ] * rtDW . jacddttya0 ) + rtB . aitunyvbo3 [ 7 ] ,
max_lateral_accel ) , - max_lateral_accel ) ; max_lateral_accel = rtB .
bkbzioulom [ 0 ] / T * ex ; T_y = rtB . bkbzioulom [ 0 ] / T * ey ; phi_d =
muDoubleScalarAsin ( muDoubleScalarMax ( muDoubleScalarMin ( T_y *
muDoubleScalarCos ( lkgf4mnmai [ 8 ] ) - max_lateral_accel *
muDoubleScalarSin ( lkgf4mnmai [ 8 ] ) , 0.99 ) , - 0.99 ) ) ; x =
muDoubleScalarAsin ( muDoubleScalarMax ( muDoubleScalarMin ( ( T_y *
muDoubleScalarCos ( lkgf4mnmai [ 8 ] ) + max_lateral_accel *
muDoubleScalarSin ( lkgf4mnmai [ 8 ] ) ) / muDoubleScalarCos ( phi_d ) , 0.99
) , - 0.99 ) ) ; if ( ! rtDW . flkg1gk54m ) { rtDW . jsubjbn3x3 = phi_d ;
rtDW . flkg1gk54m = true ; rtDW . o0rlsmhvnx = - x ; rtDW . orxmg0nmrg = true
; rtDW . lbm0o1qa52 = true ; rtDW . av31m02ta4 = true ; } phi_dot_d = ( phi_d
- rtDW . jsubjbn3x3 ) / rtB . bkbzioulom [ 23 ] ; theta_dot_d = ( - x - rtDW
. o0rlsmhvnx ) / rtB . bkbzioulom [ 23 ] ; psi_dot_d = ( 0.0 - rtDW .
mj3ycsnbqz ) / rtB . bkbzioulom [ 23 ] ; if ( ! rtDW . briiwtgmsq ) { rtDW .
bw0m2qixpq = phi_dot_d ; rtDW . briiwtgmsq = true ; rtDW . bgxyound4l =
theta_dot_d ; rtDW . p104kbp3ah = true ; rtDW . jlfkntbhq3 = psi_dot_d ; rtDW
. fdovcjpqrp = true ; } alpha = rtB . bkbzioulom [ 23 ] / ( rtB . bkbzioulom
[ 23 ] + 0.05 ) ; rtDW . bw0m2qixpq = ( 1.0 - alpha ) * rtDW . bw0m2qixpq +
alpha * phi_dot_d ; rtDW . bgxyound4l = ( 1.0 - alpha ) * rtDW . bgxyound4l +
alpha * theta_dot_d ; rtDW . jlfkntbhq3 = ( 1.0 - alpha ) * rtDW . jlfkntbhq3
+ alpha * psi_dot_d ; rtDW . jsubjbn3x3 = phi_d ; rtDW . o0rlsmhvnx = - x ;
rtDW . mj3ycsnbqz = 0.0 ; rtDW . jlqd5uv2t5 += rtB . bkbzioulom [ 23 ] ; rtB
. jxcagdmfiz [ 0 ] = phi_d ; rtB . jxcagdmfiz [ 1 ] = - x ; rtB . jxcagdmfiz
[ 2 ] = 0.0 ; rtB . mg3dqlxqgc [ 0 ] = rtDW . bw0m2qixpq ; rtB . mg3dqlxqgc [
1 ] = rtDW . bgxyound4l ; rtB . mg3dqlxqgc [ 2 ] = rtDW . jlfkntbhq3 ; rtB .
a3uqevckku [ 0 ] = ex ; rtB . a3uqevckku [ 1 ] = ey ; rtB . mdqy0sybu0 [ 0 ]
= T ; rtB . mdqy0sybu0 [ 1 ] = max_lateral_accel ; rtB . mdqy0sybu0 [ 2 ] =
T_y ; rtB . mdqy0sybu0 [ 3 ] = ez ; for ( i = 0 ; i < 6 ; i ++ ) { lkgf4mnmai
[ i ] = rtB . krtg4hkewq [ i ] ; lkgf4mnmai [ i + 6 ] = rtB . hx4zbpyorm [ i
] ; } rtDW . f3nbwloi5j = mjvsty0x4r ; ez = rtB . jxcagdmfiz [ 0 ] -
lkgf4mnmai [ 6 ] ; rtDW . isxzobxtuk += ez * rtB . bkbzioulom [ 23 ] ; rtDW .
isxzobxtuk = muDoubleScalarMax ( muDoubleScalarMin ( rtDW . isxzobxtuk , 10.0
) , - 10.0 ) ; rtB . cu04fobhdc = ( ( rtB . mg3dqlxqgc [ 0 ] - lkgf4mnmai [ 9
] ) * rtB . bkbzioulom [ 17 ] + rtB . bkbzioulom [ 14 ] * ez ) + rtB .
bkbzioulom [ 20 ] * rtDW . isxzobxtuk ; ez = rtB . jxcagdmfiz [ 1 ] -
lkgf4mnmai [ 7 ] ; rtDW . ki1qusscch += ez * rtB . bkbzioulom [ 23 ] ; rtDW .
ki1qusscch = muDoubleScalarMax ( muDoubleScalarMin ( rtDW . ki1qusscch , 10.0
) , - 10.0 ) ; rtB . h04naoubyp = ( ( rtB . mg3dqlxqgc [ 1 ] - lkgf4mnmai [
10 ] ) * rtB . bkbzioulom [ 18 ] + rtB . bkbzioulom [ 15 ] * ez ) + rtB .
bkbzioulom [ 21 ] * rtDW . ki1qusscch ; ez = rtB . jxcagdmfiz [ 2 ] -
lkgf4mnmai [ 8 ] ; rtDW . aswygtmbgv += ez * rtB . bkbzioulom [ 23 ] ; rtDW .
aswygtmbgv = muDoubleScalarMax ( muDoubleScalarMin ( rtDW . aswygtmbgv , 10.0
) , - 10.0 ) ; rtB . dzp0ufmz11 = ( ( rtB . mg3dqlxqgc [ 2 ] - lkgf4mnmai [
11 ] ) * rtB . bkbzioulom [ 19 ] + rtB . bkbzioulom [ 16 ] * ez ) + rtB .
bkbzioulom [ 22 ] * rtDW . aswygtmbgv ; } for ( i = 0 ; i < 6 ; i ++ ) {
lkgf4mnmai [ i ] = rtB . krtg4hkewq [ i ] ; lkgf4mnmai [ i + 6 ] = rtB .
hx4zbpyorm [ i ] ; } rtDW . d34l4kjipg = mjvsty0x4r ; rtB . ikfxwn30vz [ 0 ]
= rtB . mdqy0sybu0 [ 0 ] * rtB . mdqy0sybu0 [ 1 ] / rtB . bkbzioulom [ 0 ] ;
rtB . ikfxwn30vz [ 1 ] = rtB . mdqy0sybu0 [ 0 ] * rtB . mdqy0sybu0 [ 2 ] /
rtB . bkbzioulom [ 0 ] ; rtB . ikfxwn30vz [ 2 ] = - muDoubleScalarCos (
lkgf4mnmai [ 7 ] ) * muDoubleScalarCos ( lkgf4mnmai [ 6 ] ) * rtB .
mdqy0sybu0 [ 0 ] / rtB . bkbzioulom [ 0 ] + rtB . bkbzioulom [ 1 ] ; rtB .
ikfxwn30vz [ 3 ] = rtB . mdqy0sybu0 [ 1 ] / rtB . bkbzioulom [ 2 ] ; rtB .
ikfxwn30vz [ 4 ] = rtB . mdqy0sybu0 [ 2 ] / rtB . bkbzioulom [ 3 ] ; rtB .
ikfxwn30vz [ 5 ] = rtB . mdqy0sybu0 [ 3 ] / rtB . bkbzioulom [ 4 ] ; if (
ssIsSampleHit ( rtS , 1 , 0 ) ) { } rtB . nwsu2llks5 = rtB . aitunyvbo3 [ 2 ]
- rtB . krtg4hkewq [ 2 ] ; if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { { if ( rtDW
. fsz1bpdsia . AQHandles && ssGetLogOutput ( rtS ) ) { sdiWriteSignal ( rtDW
. fsz1bpdsia . AQHandles , ssGetTaskTime ( rtS , 1 ) , ( char * ) & rtB .
krtg4hkewq [ 2 ] + 0 ) ; } } } UNUSED_PARAMETER ( tid ) ; } void
MdlOutputsTID2 ( int_T tid ) { UNUSED_PARAMETER ( tid ) ; } void MdlUpdate (
int_T tid ) { UNUSED_PARAMETER ( tid ) ; } void MdlUpdateTID2 ( int_T tid ) {
UNUSED_PARAMETER ( tid ) ; } void MdlDerivatives ( void ) { XDot * _rtXdot ;
int32_T i ; _rtXdot = ( ( XDot * ) ssGetdX ( rtS ) ) ; for ( i = 0 ; i < 6 ;
i ++ ) { _rtXdot -> cm1oq22mti [ i ] = rtB . hx4zbpyorm [ i ] ; _rtXdot ->
ffddlbb5mp [ i ] = rtB . ikfxwn30vz [ i ] ; } } void MdlProjection ( void ) {
} void MdlTerminate ( void ) { { if ( rtDW . fsz1bpdsia . AQHandles ) {
sdiTerminateStreaming ( & rtDW . fsz1bpdsia . AQHandles ) ; } } } static void
mr_smlnk_cacheDataAsMxArray ( mxArray * destArray , mwIndex i , int j , const
void * srcData , size_t numBytes ) ; static void mr_smlnk_cacheDataAsMxArray
( mxArray * destArray , mwIndex i , int j , const void * srcData , size_t
numBytes ) { mxArray * newArray = mxCreateUninitNumericMatrix ( ( size_t ) 1
, numBytes , mxUINT8_CLASS , mxREAL ) ; memcpy ( ( uint8_T * ) mxGetData (
newArray ) , ( const uint8_T * ) srcData , numBytes ) ; mxSetFieldByNumber (
destArray , i , j , newArray ) ; } static void
mr_smlnk_restoreDataFromMxArray ( void * destData , const mxArray * srcArray
, mwIndex i , int j , size_t numBytes ) ; static void
mr_smlnk_restoreDataFromMxArray ( void * destData , const mxArray * srcArray
, mwIndex i , int j , size_t numBytes ) { memcpy ( ( uint8_T * ) destData , (
const uint8_T * ) mxGetData ( mxGetFieldByNumber ( srcArray , i , j ) ) ,
numBytes ) ; } static void mr_smlnk_cacheBitFieldToMxArray ( mxArray *
destArray , mwIndex i , int j , uint_T bitVal ) ; static void
mr_smlnk_cacheBitFieldToMxArray ( mxArray * destArray , mwIndex i , int j ,
uint_T bitVal ) { mxSetFieldByNumber ( destArray , i , j ,
mxCreateDoubleScalar ( ( real_T ) bitVal ) ) ; } static uint_T
mr_smlnk_extractBitFieldFromMxArray ( const mxArray * srcArray , mwIndex i ,
int j , uint_T numBits ) ; static uint_T mr_smlnk_extractBitFieldFromMxArray
( const mxArray * srcArray , mwIndex i , int j , uint_T numBits ) { const
uint_T varVal = ( uint_T ) mxGetScalar ( mxGetFieldByNumber ( srcArray , i ,
j ) ) ; return varVal & ( ( 1u << numBits ) - 1u ) ; } static void
mr_smlnk_cacheDataToMxArrayWithOffset ( mxArray * destArray , mwIndex i , int
j , mwIndex offset , const void * srcData , size_t numBytes ) ; static void
mr_smlnk_cacheDataToMxArrayWithOffset ( mxArray * destArray , mwIndex i , int
j , mwIndex offset , const void * srcData , size_t numBytes ) { uint8_T *
varData = ( uint8_T * ) mxGetData ( mxGetFieldByNumber ( destArray , i , j )
) ; memcpy ( ( uint8_T * ) & varData [ offset * numBytes ] , ( const uint8_T
* ) srcData , numBytes ) ; } static void
mr_smlnk_restoreDataFromMxArrayWithOffset ( void * destData , const mxArray *
srcArray , mwIndex i , int j , mwIndex offset , size_t numBytes ) ; static
void mr_smlnk_restoreDataFromMxArrayWithOffset ( void * destData , const
mxArray * srcArray , mwIndex i , int j , mwIndex offset , size_t numBytes ) {
const uint8_T * varData = ( const uint8_T * ) mxGetData ( mxGetFieldByNumber
( srcArray , i , j ) ) ; memcpy ( ( uint8_T * ) destData , ( const uint8_T *
) & varData [ offset * numBytes ] , numBytes ) ; } static void
mr_smlnk_cacheBitFieldToCellArrayWithOffset ( mxArray * destArray , mwIndex i
, int j , mwIndex offset , uint_T fieldVal ) ; static void
mr_smlnk_cacheBitFieldToCellArrayWithOffset ( mxArray * destArray , mwIndex i
, int j , mwIndex offset , uint_T fieldVal ) { mxSetCell ( mxGetFieldByNumber
( destArray , i , j ) , offset , mxCreateDoubleScalar ( ( real_T ) fieldVal )
) ; } static uint_T mr_smlnk_extractBitFieldFromCellArrayWithOffset ( const
mxArray * srcArray , mwIndex i , int j , mwIndex offset , uint_T numBits ) ;
static uint_T mr_smlnk_extractBitFieldFromCellArrayWithOffset ( const mxArray
* srcArray , mwIndex i , int j , mwIndex offset , uint_T numBits ) { const
uint_T fieldVal = ( uint_T ) mxGetScalar ( mxGetCell ( mxGetFieldByNumber (
srcArray , i , j ) , offset ) ) ; return fieldVal & ( ( 1u << numBits ) - 1u
) ; } mxArray * mr_smlnk_GetDWork ( ) { static const char_T * ssDWFieldNames
[ 3 ] = { "rtB" , "rtDW" , "NULL_PrevZCX" , } ; mxArray * ssDW =
mxCreateStructMatrix ( 1 , 1 , 3 , ssDWFieldNames ) ;
mr_smlnk_cacheDataAsMxArray ( ssDW , 0 , 0 , ( const void * ) & ( rtB ) ,
sizeof ( rtB ) ) ; { static const char_T * rtdwDataFieldNames [ 36 ] = {
"rtDW.pydj2pulvz" , "rtDW.h4kbtduxza" , "rtDW.jacddttya0" , "rtDW.jsubjbn3x3"
, "rtDW.o0rlsmhvnx" , "rtDW.mj3ycsnbqz" , "rtDW.jlqd5uv2t5" ,
"rtDW.bw0m2qixpq" , "rtDW.bgxyound4l" , "rtDW.jlfkntbhq3" , "rtDW.isxzobxtuk"
, "rtDW.ki1qusscch" , "rtDW.aswygtmbgv" , "rtDW.e3y0cazldt" ,
"rtDW.ejujdun315" , "rtDW.f2x2yaoqgy" , "rtDW.d34l4kjipg" , "rtDW.f3nbwloi5j"
, "rtDW.njpxizyule" , "rtDW.cjhm3acixb" , "rtDW.lxythyf1cb" ,
"rtDW.nh4tvjogpv" , "rtDW.ba1rd1ajhl" , "rtDW.flkg1gk54m" , "rtDW.orxmg0nmrg"
, "rtDW.lbm0o1qa52" , "rtDW.av31m02ta4" , "rtDW.briiwtgmsq" ,
"rtDW.p104kbp3ah" , "rtDW.fdovcjpqrp" , "rtDW.itne4ou4c4" , "rtDW.ffn3ukflan"
, "rtDW.a0uid4l0f4" , "rtDW.i5lo0j2nbo" , "rtDW.jwsyzsowec" ,
"rtDW.lnsdae4spp" , } ; mxArray * rtdwData = mxCreateStructMatrix ( 1 , 1 ,
36 , rtdwDataFieldNames ) ; mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 0 ,
( const void * ) & ( rtDW . pydj2pulvz ) , sizeof ( rtDW . pydj2pulvz ) ) ;
mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 1 , ( const void * ) & ( rtDW .
h4kbtduxza ) , sizeof ( rtDW . h4kbtduxza ) ) ; mr_smlnk_cacheDataAsMxArray (
rtdwData , 0 , 2 , ( const void * ) & ( rtDW . jacddttya0 ) , sizeof ( rtDW .
jacddttya0 ) ) ; mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 3 , ( const
void * ) & ( rtDW . jsubjbn3x3 ) , sizeof ( rtDW . jsubjbn3x3 ) ) ;
mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 4 , ( const void * ) & ( rtDW .
o0rlsmhvnx ) , sizeof ( rtDW . o0rlsmhvnx ) ) ; mr_smlnk_cacheDataAsMxArray (
rtdwData , 0 , 5 , ( const void * ) & ( rtDW . mj3ycsnbqz ) , sizeof ( rtDW .
mj3ycsnbqz ) ) ; mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 6 , ( const
void * ) & ( rtDW . jlqd5uv2t5 ) , sizeof ( rtDW . jlqd5uv2t5 ) ) ;
mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 7 , ( const void * ) & ( rtDW .
bw0m2qixpq ) , sizeof ( rtDW . bw0m2qixpq ) ) ; mr_smlnk_cacheDataAsMxArray (
rtdwData , 0 , 8 , ( const void * ) & ( rtDW . bgxyound4l ) , sizeof ( rtDW .
bgxyound4l ) ) ; mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 9 , ( const
void * ) & ( rtDW . jlfkntbhq3 ) , sizeof ( rtDW . jlfkntbhq3 ) ) ;
mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 10 , ( const void * ) & ( rtDW .
isxzobxtuk ) , sizeof ( rtDW . isxzobxtuk ) ) ; mr_smlnk_cacheDataAsMxArray (
rtdwData , 0 , 11 , ( const void * ) & ( rtDW . ki1qusscch ) , sizeof ( rtDW
. ki1qusscch ) ) ; mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 12 , ( const
void * ) & ( rtDW . aswygtmbgv ) , sizeof ( rtDW . aswygtmbgv ) ) ;
mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 13 , ( const void * ) & ( rtDW .
e3y0cazldt ) , sizeof ( rtDW . e3y0cazldt ) ) ; mr_smlnk_cacheDataAsMxArray (
rtdwData , 0 , 14 , ( const void * ) & ( rtDW . ejujdun315 ) , sizeof ( rtDW
. ejujdun315 ) ) ; mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 15 , ( const
void * ) & ( rtDW . f2x2yaoqgy ) , sizeof ( rtDW . f2x2yaoqgy ) ) ;
mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 16 , ( const void * ) & ( rtDW .
d34l4kjipg ) , sizeof ( rtDW . d34l4kjipg ) ) ; mr_smlnk_cacheDataAsMxArray (
rtdwData , 0 , 17 , ( const void * ) & ( rtDW . f3nbwloi5j ) , sizeof ( rtDW
. f3nbwloi5j ) ) ; mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 18 , ( const
void * ) & ( rtDW . njpxizyule ) , sizeof ( rtDW . njpxizyule ) ) ;
mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 19 , ( const void * ) & ( rtDW .
cjhm3acixb ) , sizeof ( rtDW . cjhm3acixb ) ) ; mr_smlnk_cacheDataAsMxArray (
rtdwData , 0 , 20 , ( const void * ) & ( rtDW . lxythyf1cb ) , sizeof ( rtDW
. lxythyf1cb ) ) ; mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 21 , ( const
void * ) & ( rtDW . nh4tvjogpv ) , sizeof ( rtDW . nh4tvjogpv ) ) ;
mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 22 , ( const void * ) & ( rtDW .
ba1rd1ajhl ) , sizeof ( rtDW . ba1rd1ajhl ) ) ; mr_smlnk_cacheDataAsMxArray (
rtdwData , 0 , 23 , ( const void * ) & ( rtDW . flkg1gk54m ) , sizeof ( rtDW
. flkg1gk54m ) ) ; mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 24 , ( const
void * ) & ( rtDW . orxmg0nmrg ) , sizeof ( rtDW . orxmg0nmrg ) ) ;
mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 25 , ( const void * ) & ( rtDW .
lbm0o1qa52 ) , sizeof ( rtDW . lbm0o1qa52 ) ) ; mr_smlnk_cacheDataAsMxArray (
rtdwData , 0 , 26 , ( const void * ) & ( rtDW . av31m02ta4 ) , sizeof ( rtDW
. av31m02ta4 ) ) ; mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 27 , ( const
void * ) & ( rtDW . briiwtgmsq ) , sizeof ( rtDW . briiwtgmsq ) ) ;
mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 28 , ( const void * ) & ( rtDW .
p104kbp3ah ) , sizeof ( rtDW . p104kbp3ah ) ) ; mr_smlnk_cacheDataAsMxArray (
rtdwData , 0 , 29 , ( const void * ) & ( rtDW . fdovcjpqrp ) , sizeof ( rtDW
. fdovcjpqrp ) ) ; mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 30 , ( const
void * ) & ( rtDW . itne4ou4c4 ) , sizeof ( rtDW . itne4ou4c4 ) ) ;
mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 31 , ( const void * ) & ( rtDW .
ffn3ukflan ) , sizeof ( rtDW . ffn3ukflan ) ) ; mr_smlnk_cacheDataAsMxArray (
rtdwData , 0 , 32 , ( const void * ) & ( rtDW . a0uid4l0f4 ) , sizeof ( rtDW
. a0uid4l0f4 ) ) ; mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 33 , ( const
void * ) & ( rtDW . i5lo0j2nbo ) , sizeof ( rtDW . i5lo0j2nbo ) ) ;
mr_smlnk_cacheDataAsMxArray ( rtdwData , 0 , 34 , ( const void * ) & ( rtDW .
jwsyzsowec ) , sizeof ( rtDW . jwsyzsowec ) ) ; mr_smlnk_cacheDataAsMxArray (
rtdwData , 0 , 35 , ( const void * ) & ( rtDW . lnsdae4spp ) , sizeof ( rtDW
. lnsdae4spp ) ) ; mxSetFieldByNumber ( ssDW , 0 , 1 , rtdwData ) ; } return
ssDW ; } void mr_smlnk_SetDWork ( const mxArray * ssDW ) { ( void ) ssDW ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtB ) , ssDW , 0 , 0 ,
sizeof ( rtB ) ) ; { const mxArray * rtdwData = mxGetFieldByNumber ( ssDW , 0
, 1 ) ; mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . pydj2pulvz )
, rtdwData , 0 , 0 , sizeof ( rtDW . pydj2pulvz ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . h4kbtduxza ) ,
rtdwData , 0 , 1 , sizeof ( rtDW . h4kbtduxza ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . jacddttya0 ) ,
rtdwData , 0 , 2 , sizeof ( rtDW . jacddttya0 ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . jsubjbn3x3 ) ,
rtdwData , 0 , 3 , sizeof ( rtDW . jsubjbn3x3 ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . o0rlsmhvnx ) ,
rtdwData , 0 , 4 , sizeof ( rtDW . o0rlsmhvnx ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . mj3ycsnbqz ) ,
rtdwData , 0 , 5 , sizeof ( rtDW . mj3ycsnbqz ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . jlqd5uv2t5 ) ,
rtdwData , 0 , 6 , sizeof ( rtDW . jlqd5uv2t5 ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . bw0m2qixpq ) ,
rtdwData , 0 , 7 , sizeof ( rtDW . bw0m2qixpq ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . bgxyound4l ) ,
rtdwData , 0 , 8 , sizeof ( rtDW . bgxyound4l ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . jlfkntbhq3 ) ,
rtdwData , 0 , 9 , sizeof ( rtDW . jlfkntbhq3 ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . isxzobxtuk ) ,
rtdwData , 0 , 10 , sizeof ( rtDW . isxzobxtuk ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . ki1qusscch ) ,
rtdwData , 0 , 11 , sizeof ( rtDW . ki1qusscch ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . aswygtmbgv ) ,
rtdwData , 0 , 12 , sizeof ( rtDW . aswygtmbgv ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . e3y0cazldt ) ,
rtdwData , 0 , 13 , sizeof ( rtDW . e3y0cazldt ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . ejujdun315 ) ,
rtdwData , 0 , 14 , sizeof ( rtDW . ejujdun315 ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . f2x2yaoqgy ) ,
rtdwData , 0 , 15 , sizeof ( rtDW . f2x2yaoqgy ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . d34l4kjipg ) ,
rtdwData , 0 , 16 , sizeof ( rtDW . d34l4kjipg ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . f3nbwloi5j ) ,
rtdwData , 0 , 17 , sizeof ( rtDW . f3nbwloi5j ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . njpxizyule ) ,
rtdwData , 0 , 18 , sizeof ( rtDW . njpxizyule ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . cjhm3acixb ) ,
rtdwData , 0 , 19 , sizeof ( rtDW . cjhm3acixb ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . lxythyf1cb ) ,
rtdwData , 0 , 20 , sizeof ( rtDW . lxythyf1cb ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . nh4tvjogpv ) ,
rtdwData , 0 , 21 , sizeof ( rtDW . nh4tvjogpv ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . ba1rd1ajhl ) ,
rtdwData , 0 , 22 , sizeof ( rtDW . ba1rd1ajhl ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . flkg1gk54m ) ,
rtdwData , 0 , 23 , sizeof ( rtDW . flkg1gk54m ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . orxmg0nmrg ) ,
rtdwData , 0 , 24 , sizeof ( rtDW . orxmg0nmrg ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . lbm0o1qa52 ) ,
rtdwData , 0 , 25 , sizeof ( rtDW . lbm0o1qa52 ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . av31m02ta4 ) ,
rtdwData , 0 , 26 , sizeof ( rtDW . av31m02ta4 ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . briiwtgmsq ) ,
rtdwData , 0 , 27 , sizeof ( rtDW . briiwtgmsq ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . p104kbp3ah ) ,
rtdwData , 0 , 28 , sizeof ( rtDW . p104kbp3ah ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . fdovcjpqrp ) ,
rtdwData , 0 , 29 , sizeof ( rtDW . fdovcjpqrp ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . itne4ou4c4 ) ,
rtdwData , 0 , 30 , sizeof ( rtDW . itne4ou4c4 ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . ffn3ukflan ) ,
rtdwData , 0 , 31 , sizeof ( rtDW . ffn3ukflan ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . a0uid4l0f4 ) ,
rtdwData , 0 , 32 , sizeof ( rtDW . a0uid4l0f4 ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . i5lo0j2nbo ) ,
rtdwData , 0 , 33 , sizeof ( rtDW . i5lo0j2nbo ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . jwsyzsowec ) ,
rtdwData , 0 , 34 , sizeof ( rtDW . jwsyzsowec ) ) ;
mr_smlnk_restoreDataFromMxArray ( ( void * ) & ( rtDW . lnsdae4spp ) ,
rtdwData , 0 , 35 , sizeof ( rtDW . lnsdae4spp ) ) ; } } mxArray *
mr_smlnk_GetSimStateDisallowedBlocks ( ) { mxArray * data =
mxCreateCellMatrix ( 9 , 3 ) ; mwIndex subs [ 2 ] , offset ; { static const
char_T * blockType [ 9 ] = { "Scope" , "Scope" , "Scope" , "Scope" , "Scope"
, "Scope" , "Scope" , "Scope" , "Scope" , } ; static const char_T * blockPath
[ 9 ] = { "smlnk/Angles Phi, Theta, Psi" ,
"smlnk/Complete Lin. Trajectory Generation" , "smlnk/Complete Output" ,
"smlnk/Coordinates x,y,z" , "smlnk/Height error" ,
"smlnk/Position & Trajectory Comparison" , "smlnk/Scope4" ,
"smlnk/Thrust Control" , "smlnk/Ux Uy controls" , } ; static const int reason
[ 9 ] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , } ; for ( subs [ 0 ] = 0 ; subs
[ 0 ] < 9 ; ++ ( subs [ 0 ] ) ) { subs [ 1 ] = 0 ; offset =
mxCalcSingleSubscript ( data , 2 , subs ) ; mxSetCell ( data , offset ,
mxCreateString ( blockType [ subs [ 0 ] ] ) ) ; subs [ 1 ] = 1 ; offset =
mxCalcSingleSubscript ( data , 2 , subs ) ; mxSetCell ( data , offset ,
mxCreateString ( blockPath [ subs [ 0 ] ] ) ) ; subs [ 1 ] = 2 ; offset =
mxCalcSingleSubscript ( data , 2 , subs ) ; mxSetCell ( data , offset ,
mxCreateDoubleScalar ( ( real_T ) reason [ subs [ 0 ] ] ) ) ; } } return data
; } void MdlInitializeSizes ( void ) { ssSetNumContStates ( rtS , 12 ) ;
ssSetNumPeriodicContStates ( rtS , 0 ) ; ssSetNumY ( rtS , 0 ) ; ssSetNumU (
rtS , 0 ) ; ssSetDirectFeedThrough ( rtS , 0 ) ; ssSetNumSampleTimes ( rtS ,
2 ) ; ssSetNumBlocks ( rtS , 29 ) ; ssSetNumBlockIO ( rtS , 14 ) ;
ssSetNumBlockParams ( rtS , 13 ) ; } void MdlInitializeSampleTimes ( void ) {
ssSetSampleTime ( rtS , 0 , 0.0 ) ; ssSetSampleTime ( rtS , 1 , 0.0001 ) ;
ssSetOffsetTime ( rtS , 0 , 0.0 ) ; ssSetOffsetTime ( rtS , 1 , 0.0 ) ; }
void raccel_set_checksum ( ) { ssSetChecksumVal ( rtS , 0 , 1694352703U ) ;
ssSetChecksumVal ( rtS , 1 , 1627899302U ) ; ssSetChecksumVal ( rtS , 2 ,
262459659U ) ; ssSetChecksumVal ( rtS , 3 , 2132616286U ) ; }
#if defined(_MSC_VER)
#pragma optimize( "", off )
#endif
SimStruct * raccel_register_model ( ssExecutionInfo * executionInfo ) {
static struct _ssMdlInfo mdlInfo ; static struct _ssBlkInfo2 blkInfo2 ;
static struct _ssBlkInfoSLSize blkInfoSLSize ; rt_modelMapInfoPtr = & (
rt_dataMapInfo . mmi ) ; executionInfo -> gblObjects_ . numToFiles = 0 ;
executionInfo -> gblObjects_ . numFrFiles = 0 ; executionInfo -> gblObjects_
. numFrWksBlocks = 0 ; executionInfo -> gblObjects_ . numModelInputs = 0 ;
executionInfo -> gblObjects_ . numRootInportBlks = 0 ; executionInfo ->
gblObjects_ . inportDataTypeIdx = NULL ; executionInfo -> gblObjects_ .
inportDims = NULL ; executionInfo -> gblObjects_ . inportComplex = NULL ;
executionInfo -> gblObjects_ . inportInterpoFlag = NULL ; executionInfo ->
gblObjects_ . inportContinuous = NULL ; ( void ) memset ( ( char_T * ) rtS ,
0 , sizeof ( SimStruct ) ) ; ( void ) memset ( ( char_T * ) & mdlInfo , 0 ,
sizeof ( struct _ssMdlInfo ) ) ; ( void ) memset ( ( char_T * ) & blkInfo2 ,
0 , sizeof ( struct _ssBlkInfo2 ) ) ; ( void ) memset ( ( char_T * ) &
blkInfoSLSize , 0 , sizeof ( struct _ssBlkInfoSLSize ) ) ; ssSetBlkInfo2Ptr (
rtS , & blkInfo2 ) ; ssSetBlkInfoSLSizePtr ( rtS , & blkInfoSLSize ) ;
ssSetMdlInfoPtr ( rtS , & mdlInfo ) ; ssSetExecutionInfo ( rtS ,
executionInfo ) ; slsaAllocOPModelData ( rtS ) ; { static time_T mdlPeriod [
NSAMPLE_TIMES ] ; static time_T mdlOffset [ NSAMPLE_TIMES ] ; static time_T
mdlTaskTimes [ NSAMPLE_TIMES ] ; static int_T mdlTsMap [ NSAMPLE_TIMES ] ;
static int_T mdlSampleHits [ NSAMPLE_TIMES ] ; static boolean_T
mdlTNextWasAdjustedPtr [ NSAMPLE_TIMES ] ; static int_T mdlPerTaskSampleHits
[ NSAMPLE_TIMES * NSAMPLE_TIMES ] ; static time_T mdlTimeOfNextSampleHit [
NSAMPLE_TIMES ] ; { int_T i ; for ( i = 0 ; i < NSAMPLE_TIMES ; i ++ ) {
mdlPeriod [ i ] = 0.0 ; mdlOffset [ i ] = 0.0 ; mdlTaskTimes [ i ] = 0.0 ;
mdlTsMap [ i ] = i ; mdlSampleHits [ i ] = 1 ; } } ssSetSampleTimePtr ( rtS ,
& mdlPeriod [ 0 ] ) ; ssSetOffsetTimePtr ( rtS , & mdlOffset [ 0 ] ) ;
ssSetSampleTimeTaskIDPtr ( rtS , & mdlTsMap [ 0 ] ) ; ssSetTPtr ( rtS , &
mdlTaskTimes [ 0 ] ) ; ssSetSampleHitPtr ( rtS , & mdlSampleHits [ 0 ] ) ;
ssSetTNextWasAdjustedPtr ( rtS , & mdlTNextWasAdjustedPtr [ 0 ] ) ;
ssSetPerTaskSampleHitsPtr ( rtS , & mdlPerTaskSampleHits [ 0 ] ) ;
ssSetTimeOfNextSampleHitPtr ( rtS , & mdlTimeOfNextSampleHit [ 0 ] ) ; }
ssSetSolverMode ( rtS , SOLVER_MODE_SINGLETASKING ) ; { ssSetBlockIO ( rtS ,
( ( void * ) & rtB ) ) ; ( void ) memset ( ( ( void * ) & rtB ) , 0 , sizeof
( B ) ) ; } { real_T * x = ( real_T * ) & rtX ; ssSetContStates ( rtS , x ) ;
( void ) memset ( ( void * ) x , 0 , sizeof ( X ) ) ; } { void * dwork = (
void * ) & rtDW ; ssSetRootDWork ( rtS , dwork ) ; ( void ) memset ( dwork ,
0 , sizeof ( DW ) ) ; } { static DataTypeTransInfo dtInfo ; ( void ) memset (
( char_T * ) & dtInfo , 0 , sizeof ( dtInfo ) ) ; ssSetModelMappingInfo ( rtS
, & dtInfo ) ; dtInfo . numDataTypes = 23 ; dtInfo . dataTypeSizes = &
rtDataTypeSizes [ 0 ] ; dtInfo . dataTypeNames = & rtDataTypeNames [ 0 ] ;
dtInfo . BTransTable = & rtBTransTable ; dtInfo . PTransTable = &
rtPTransTable ; dtInfo . dataTypeInfoTable = rtDataTypeInfoTable ; }
smlnk_InitializeDataMapInfo ( ) ; ssSetIsRapidAcceleratorActive ( rtS , true
) ; ssSetRootSS ( rtS , rtS ) ; ssSetVersion ( rtS , SIMSTRUCT_VERSION_LEVEL2
) ; ssSetModelName ( rtS , "smlnk" ) ; ssSetPath ( rtS , "smlnk" ) ;
ssSetTStart ( rtS , 0.0 ) ; ssSetTFinal ( rtS , 120.0 ) ; ssSetStepSize ( rtS
, 0.0001 ) ; ssSetFixedStepSize ( rtS , 0.0001 ) ; { static RTWLogInfo
rt_DataLoggingInfo ; rt_DataLoggingInfo . loggingInterval = ( NULL ) ;
ssSetRTWLogInfo ( rtS , & rt_DataLoggingInfo ) ; } { { static int_T
rt_LoggedStateWidths [ ] = { 6 , 6 } ; static int_T
rt_LoggedStateNumDimensions [ ] = { 1 , 1 } ; static int_T
rt_LoggedStateDimensions [ ] = { 6 , 6 } ; static boolean_T
rt_LoggedStateIsVarDims [ ] = { 0 , 0 } ; static BuiltInDTypeId
rt_LoggedStateDataTypeIds [ ] = { SS_DOUBLE , SS_DOUBLE } ; static int_T
rt_LoggedStateComplexSignals [ ] = { 0 , 0 } ; static RTWPreprocessingFcnPtr
rt_LoggingStatePreprocessingFcnPtrs [ ] = { ( NULL ) , ( NULL ) } ; static
const char_T * rt_LoggedStateLabels [ ] = { "CSTATE" , "CSTATE" } ; static
const char_T * rt_LoggedStateBlockNames [ ] = { "smlnk/  " , "smlnk/ " } ;
static const char_T * rt_LoggedStateNames [ ] = { "" , "" } ; static
boolean_T rt_LoggedStateCrossMdlRef [ ] = { 0 , 0 } ; static
RTWLogDataTypeConvert rt_RTWLogDataTypeConvert [ ] = { { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } } ; static int_T rt_LoggedStateIdxList [ ] = { 0 , 1 }
; static RTWLogSignalInfo rt_LoggedStateSignalInfo = { 2 ,
rt_LoggedStateWidths , rt_LoggedStateNumDimensions , rt_LoggedStateDimensions
, rt_LoggedStateIsVarDims , ( NULL ) , ( NULL ) , rt_LoggedStateDataTypeIds ,
rt_LoggedStateComplexSignals , ( NULL ) , rt_LoggingStatePreprocessingFcnPtrs
, { rt_LoggedStateLabels } , ( NULL ) , ( NULL ) , ( NULL ) , {
rt_LoggedStateBlockNames } , { rt_LoggedStateNames } ,
rt_LoggedStateCrossMdlRef , rt_RTWLogDataTypeConvert , rt_LoggedStateIdxList
} ; static void * rt_LoggedStateSignalPtrs [ 2 ] ; rtliSetLogXSignalPtrs (
ssGetRTWLogInfo ( rtS ) , ( LogSignalPtrsType ) rt_LoggedStateSignalPtrs ) ;
rtliSetLogXSignalInfo ( ssGetRTWLogInfo ( rtS ) , & rt_LoggedStateSignalInfo
) ; rt_LoggedStateSignalPtrs [ 0 ] = ( void * ) & rtX . cm1oq22mti [ 0 ] ;
rt_LoggedStateSignalPtrs [ 1 ] = ( void * ) & rtX . ffddlbb5mp [ 0 ] ; }
rtliSetLogT ( ssGetRTWLogInfo ( rtS ) , "tout" ) ; rtliSetLogX (
ssGetRTWLogInfo ( rtS ) , "" ) ; rtliSetLogXFinal ( ssGetRTWLogInfo ( rtS ) ,
"xFinal" ) ; rtliSetLogVarNameModifier ( ssGetRTWLogInfo ( rtS ) , "none" ) ;
rtliSetLogFormat ( ssGetRTWLogInfo ( rtS ) , 4 ) ; rtliSetLogMaxRows (
ssGetRTWLogInfo ( rtS ) , 0 ) ; rtliSetLogDecimation ( ssGetRTWLogInfo ( rtS
) , 1 ) ; rtliSetLogY ( ssGetRTWLogInfo ( rtS ) , "" ) ;
rtliSetLogYSignalInfo ( ssGetRTWLogInfo ( rtS ) , ( NULL ) ) ;
rtliSetLogYSignalPtrs ( ssGetRTWLogInfo ( rtS ) , ( NULL ) ) ; } { static
struct _ssStatesInfo2 statesInfo2 ; ssSetStatesInfo2 ( rtS , & statesInfo2 )
; } { static ssPeriodicStatesInfo periodicStatesInfo ;
ssSetPeriodicStatesInfo ( rtS , & periodicStatesInfo ) ; } { static
ssJacobianPerturbationBounds jacobianPerturbationBounds ;
ssSetJacobianPerturbationBounds ( rtS , & jacobianPerturbationBounds ) ; } {
static ssSolverInfo slvrInfo ; static boolean_T contStatesDisabled [ 12 ] ;
static ssNonContDerivSigInfo nonContDerivSigInfo [ 7 ] = { { 3 * sizeof (
real_T ) , ( char * ) ( & rtB . mg3dqlxqgc [ 0 ] ) , ( NULL ) } , { 3 *
sizeof ( real_T ) , ( char * ) ( & rtB . jxcagdmfiz [ 0 ] ) , ( NULL ) } , {
4 * sizeof ( real_T ) , ( char * ) ( & rtB . mdqy0sybu0 [ 0 ] ) , ( NULL ) }
, { 24 * sizeof ( real_T ) , ( char * ) ( & rtB . bkbzioulom [ 0 ] ) , ( NULL
) } , { 1 * sizeof ( real_T ) , ( char * ) ( & rtB . dzp0ufmz11 ) , ( NULL )
} , { 1 * sizeof ( real_T ) , ( char * ) ( & rtB . h04naoubyp ) , ( NULL ) }
, { 1 * sizeof ( real_T ) , ( char * ) ( & rtB . cu04fobhdc ) , ( NULL ) } }
; ssSetNumNonContDerivSigInfos ( rtS , 7 ) ; ssSetNonContDerivSigInfos ( rtS
, nonContDerivSigInfo ) ; ssSetSolverInfo ( rtS , & slvrInfo ) ;
ssSetSolverName ( rtS , "ode4" ) ; ssSetVariableStepSolver ( rtS , 0 ) ;
ssSetSolverConsistencyChecking ( rtS , 0 ) ; ssSetSolverAdaptiveZcDetection (
rtS , 0 ) ; ssSetSolverRobustResetMethod ( rtS , 0 ) ;
ssSetSolverStateProjection ( rtS , 0 ) ; ssSetSolverMassMatrixType ( rtS , (
ssMatrixType ) 0 ) ; ssSetSolverMassMatrixNzMax ( rtS , 0 ) ;
ssSetModelOutputs ( rtS , MdlOutputs ) ; ssSetModelUpdate ( rtS , MdlUpdate )
; ssSetModelDerivatives ( rtS , MdlDerivatives ) ; ssSetTNextTid ( rtS ,
INT_MIN ) ; ssSetTNext ( rtS , rtMinusInf ) ; ssSetSolverNeedsReset ( rtS ) ;
ssSetNumNonsampledZCs ( rtS , 0 ) ; ssSetContStateDisabled ( rtS ,
contStatesDisabled ) ; } ssSetChecksumVal ( rtS , 0 , 1694352703U ) ;
ssSetChecksumVal ( rtS , 1 , 1627899302U ) ; ssSetChecksumVal ( rtS , 2 ,
262459659U ) ; ssSetChecksumVal ( rtS , 3 , 2132616286U ) ; { static const
sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE ; static RTWExtModeInfo
rt_ExtModeInfo ; static const sysRanDType * systemRan [ 6 ] ;
gblRTWExtModeInfo = & rt_ExtModeInfo ; ssSetRTWExtModeInfo ( rtS , &
rt_ExtModeInfo ) ; rteiSetSubSystemActiveVectorAddresses ( & rt_ExtModeInfo ,
systemRan ) ; systemRan [ 0 ] = & rtAlwaysEnabled ; systemRan [ 1 ] = &
rtAlwaysEnabled ; systemRan [ 2 ] = & rtAlwaysEnabled ; systemRan [ 3 ] = &
rtAlwaysEnabled ; systemRan [ 4 ] = & rtAlwaysEnabled ; systemRan [ 5 ] = &
rtAlwaysEnabled ; rteiSetModelMappingInfoPtr ( ssGetRTWExtModeInfo ( rtS ) ,
& ssGetModelMappingInfo ( rtS ) ) ; rteiSetChecksumsPtr ( ssGetRTWExtModeInfo
( rtS ) , ssGetChecksums ( rtS ) ) ; rteiSetTPtr ( ssGetRTWExtModeInfo ( rtS
) , ssGetTPtr ( rtS ) ) ; } slsaDisallowedBlocksForSimTargetOP ( rtS ,
mr_smlnk_GetSimStateDisallowedBlocks ) ; slsaGetWorkFcnForSimTargetOP ( rtS ,
mr_smlnk_GetDWork ) ; slsaSetWorkFcnForSimTargetOP ( rtS , mr_smlnk_SetDWork
) ; rt_RapidReadMatFileAndUpdateParams ( rtS ) ; if ( ssGetErrorStatus ( rtS
) ) { return rtS ; } executionInfo -> simulationOptions_ . stateSaveName_ =
rtliGetLogX ( ssGetRTWLogInfo ( rtS ) ) ; executionInfo -> simulationOptions_
. finalStateName_ = rtliGetLogXFinal ( ssGetRTWLogInfo ( rtS ) ) ;
executionInfo -> simulationOptions_ . outputSaveName_ = rtliGetLogY (
ssGetRTWLogInfo ( rtS ) ) ; return rtS ; }
#if defined(_MSC_VER)
#pragma optimize( "", on )
#endif
void MdlOutputsParameterSampleTime ( int_T tid ) { MdlOutputsTID2 ( tid ) ; }
