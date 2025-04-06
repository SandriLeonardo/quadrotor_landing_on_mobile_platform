#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "smlnk_capi_host.h"
#define sizeof(s) ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el) ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s) (s)
#ifndef SS_UINT64
#define SS_UINT64 17
#endif
#ifndef SS_INT64
#define SS_INT64 18
#endif
#else
#include "builtin_typeid_types.h"
#include "smlnk.h"
#include "smlnk_capi.h"
#include "smlnk_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST
#define TARGET_STRING(s)               ((NULL))
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif
static const rtwCAPI_Signals rtBlockSignals [ ] = { { 0 , 1 , TARGET_STRING (
"smlnk/Attitude Controller" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } ,
{ 1 , 1 , TARGET_STRING ( "smlnk/Attitude Controller" ) , TARGET_STRING ( ""
) , 1 , 0 , 0 , 0 , 0 } , { 2 , 1 , TARGET_STRING (
"smlnk/Attitude Controller" ) , TARGET_STRING ( "" ) , 2 , 0 , 0 , 0 , 0 } ,
{ 3 , 2 , TARGET_STRING ( "smlnk/Dynamic Model" ) , TARGET_STRING (
"Accelerations" ) , 0 , 0 , 1 , 0 , 1 } , { 4 , 3 , TARGET_STRING (
"smlnk/Parameters " ) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 5 , 4
, TARGET_STRING ( "smlnk/Position Controller" ) , TARGET_STRING ( "" ) , 0 ,
0 , 3 , 0 , 0 } , { 6 , 4 , TARGET_STRING ( "smlnk/Position Controller" ) ,
TARGET_STRING ( "Thrusts" ) , 1 , 0 , 4 , 0 , 0 } , { 7 , 4 , TARGET_STRING (
"smlnk/Position Controller" ) , TARGET_STRING ( "" ) , 2 , 0 , 5 , 0 , 0 } ,
{ 8 , 4 , TARGET_STRING ( "smlnk/Position Controller" ) , TARGET_STRING ( ""
) , 3 , 0 , 5 , 0 , 0 } , { 9 , 5 , TARGET_STRING ( "smlnk/Trajectory" ) ,
TARGET_STRING ( "Trajectory" ) , 0 , 0 , 6 , 0 , 0 } , { 10 , 0 ,
TARGET_STRING ( "smlnk/Clock" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 }
, { 11 , 0 , TARGET_STRING ( "smlnk/ " ) , TARGET_STRING ( "Velocities" ) , 0
, 0 , 1 , 0 , 1 } , { 12 , 0 , TARGET_STRING ( "smlnk/  " ) , TARGET_STRING (
"Positions" ) , 0 , 0 , 1 , 0 , 1 } , { 13 , 0 , TARGET_STRING (
"smlnk/Subtract" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 1 } , { 0 , 0 , (
NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 } } ; static const
rtwCAPI_BlockParameters rtBlockParameters [ ] = { { 14 , TARGET_STRING (
"smlnk/ " ) , TARGET_STRING ( "InitialCondition" ) , 0 , 7 , 0 } , { 15 ,
TARGET_STRING ( "smlnk/  " ) , TARGET_STRING ( "InitialCondition" ) , 0 , 7 ,
0 } , { 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 } } ; static int_T
rt_LoggedStateIdxList [ ] = { - 1 } ; static const rtwCAPI_Signals
rtRootInputs [ ] = { { 0 , 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 } } ;
static const rtwCAPI_Signals rtRootOutputs [ ] = { { 0 , 0 , ( NULL ) , (
NULL ) , 0 , 0 , 0 , 0 , 0 } } ; static const rtwCAPI_ModelParameters
rtModelParameters [ ] = { { 16 , TARGET_STRING ( "Sample_Time" ) , 0 , 0 , 0
} , { 0 , ( NULL ) , 0 , 0 , 0 } } ;
#ifndef HOST_CAPI_BUILD
static void * rtDataAddrMap [ ] = { & rtB . cu04fobhdc , & rtB . h04naoubyp ,
& rtB . dzp0ufmz11 , & rtB . ikfxwn30vz [ 0 ] , & rtB . bkbzioulom [ 0 ] , &
rtB . a3uqevckku [ 0 ] , & rtB . mdqy0sybu0 [ 0 ] , & rtB . jxcagdmfiz [ 0 ]
, & rtB . mg3dqlxqgc [ 0 ] , & rtB . aitunyvbo3 [ 0 ] , & rtB . hfcc51pulu ,
& rtB . hx4zbpyorm [ 0 ] , & rtB . krtg4hkewq [ 0 ] , & rtB . nwsu2llks5 , &
rtP . _IC_mctvldpkh3 [ 0 ] , & rtP . _IC [ 0 ] , & rtP . Sample_Time , } ;
static int32_T * rtVarDimsAddrMap [ ] = { ( NULL ) } ;
#endif
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap [ ] = { { "double" ,
"real_T" , 0 , 0 , sizeof ( real_T ) , ( uint8_T ) SS_DOUBLE , 0 , 0 , 0 } }
;
#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif
static TARGET_CONST rtwCAPI_ElementMap rtElementMap [ ] = { { ( NULL ) , 0 ,
0 , 0 , 0 } , } ; static const rtwCAPI_DimensionMap rtDimensionMap [ ] = { {
rtwCAPI_SCALAR , 0 , 2 , 0 } , { rtwCAPI_MATRIX_COL_MAJOR , 2 , 2 , 0 } , {
rtwCAPI_MATRIX_COL_MAJOR , 4 , 2 , 0 } , { rtwCAPI_MATRIX_COL_MAJOR , 6 , 2 ,
0 } , { rtwCAPI_MATRIX_COL_MAJOR , 8 , 2 , 0 } , { rtwCAPI_MATRIX_COL_MAJOR ,
10 , 2 , 0 } , { rtwCAPI_MATRIX_COL_MAJOR , 12 , 2 , 0 } , { rtwCAPI_VECTOR ,
2 , 2 , 0 } } ; static const uint_T rtDimensionArray [ ] = { 1 , 1 , 1 , 6 ,
1 , 24 , 1 , 2 , 1 , 4 , 1 , 3 , 1 , 9 } ; static const real_T
rtcapiStoredFloats [ ] = { 0.0001 , 0.0 } ; static const rtwCAPI_FixPtMap
rtFixPtMap [ ] = { { ( NULL ) , ( NULL ) , rtwCAPI_FIX_RESERVED , 0 , 0 , (
boolean_T ) 0 } , } ; static const rtwCAPI_SampleTimeMap rtSampleTimeMap [ ]
= { { ( const void * ) & rtcapiStoredFloats [ 0 ] , ( const void * ) &
rtcapiStoredFloats [ 1 ] , ( int8_T ) 1 , ( uint8_T ) 0 } , { ( const void *
) & rtcapiStoredFloats [ 1 ] , ( const void * ) & rtcapiStoredFloats [ 1 ] ,
( int8_T ) 0 , ( uint8_T ) 0 } } ; static rtwCAPI_ModelMappingStaticInfo
mmiStatic = { { rtBlockSignals , 14 , rtRootInputs , 0 , rtRootOutputs , 0 }
, { rtBlockParameters , 2 , rtModelParameters , 1 } , { ( NULL ) , 0 } , {
rtDataTypeMap , rtDimensionMap , rtFixPtMap , rtElementMap , rtSampleTimeMap
, rtDimensionArray } , "float" , { 1746000737U , 8173328U , 3270778873U ,
2862056898U } , ( NULL ) , 0 , ( boolean_T ) 0 , rt_LoggedStateIdxList } ;
const rtwCAPI_ModelMappingStaticInfo * smlnk_GetCAPIStaticMap ( void ) {
return & mmiStatic ; }
#ifndef HOST_CAPI_BUILD
void smlnk_InitializeDataMapInfo ( void ) { rtwCAPI_SetVersion ( ( *
rt_dataMapInfoPtr ) . mmi , 1 ) ; rtwCAPI_SetStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , & mmiStatic ) ; rtwCAPI_SetLoggingStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ; rtwCAPI_SetDataAddressMap ( ( *
rt_dataMapInfoPtr ) . mmi , rtDataAddrMap ) ; rtwCAPI_SetVarDimsAddressMap (
( * rt_dataMapInfoPtr ) . mmi , rtVarDimsAddrMap ) ;
rtwCAPI_SetInstanceLoggingInfo ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArray ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( ( * rt_dataMapInfoPtr ) . mmi , 0 ) ; }
#else
#ifdef __cplusplus
extern "C" {
#endif
void smlnk_host_InitializeDataMapInfo ( smlnk_host_DataMapInfo_T * dataMap ,
const char * path ) { rtwCAPI_SetVersion ( dataMap -> mmi , 1 ) ;
rtwCAPI_SetStaticMap ( dataMap -> mmi , & mmiStatic ) ;
rtwCAPI_SetDataAddressMap ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetVarDimsAddressMap ( dataMap -> mmi , ( NULL ) ) ; rtwCAPI_SetPath
( dataMap -> mmi , path ) ; rtwCAPI_SetFullPath ( dataMap -> mmi , ( NULL ) )
; rtwCAPI_SetChildMMIArray ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( dataMap -> mmi , 0 ) ; }
#ifdef __cplusplus
}
#endif
#endif
