
"use strict";

let HnrPVT = require('./HnrPVT.js');
let TimTM2 = require('./TimTM2.js');
let CfgPRT = require('./CfgPRT.js');
let CfgCFG = require('./CfgCFG.js');
let NavSBAS = require('./NavSBAS.js');
let MonHW = require('./MonHW.js');
let NavTIMEGPS = require('./NavTIMEGPS.js');
let CfgDAT = require('./CfgDAT.js');
let CfgSBAS = require('./CfgSBAS.js');
let MgaGAL = require('./MgaGAL.js');
let MonVER_Extension = require('./MonVER_Extension.js');
let UpdSOS = require('./UpdSOS.js');
let CfgGNSS = require('./CfgGNSS.js');
let RxmRAW = require('./RxmRAW.js');
let MonHW6 = require('./MonHW6.js');
let RxmRAWX_Meas = require('./RxmRAWX_Meas.js');
let EsfRAW_Block = require('./EsfRAW_Block.js');
let NavSTATUS = require('./NavSTATUS.js');
let NavTIMEUTC = require('./NavTIMEUTC.js');
let Inf = require('./Inf.js');
let NavVELECEF = require('./NavVELECEF.js');
let RxmRAW_SV = require('./RxmRAW_SV.js');
let RxmALM = require('./RxmALM.js');
let CfgMSG = require('./CfgMSG.js');
let NavSBAS_SV = require('./NavSBAS_SV.js');
let AidHUI = require('./AidHUI.js');
let NavPVT7 = require('./NavPVT7.js');
let EsfRAW = require('./EsfRAW.js');
let MonGNSS = require('./MonGNSS.js');
let AidALM = require('./AidALM.js');
let NavVELNED = require('./NavVELNED.js');
let CfgNAV5 = require('./CfgNAV5.js');
let NavDGPS = require('./NavDGPS.js');
let AidEPH = require('./AidEPH.js');
let CfgHNR = require('./CfgHNR.js');
let NavPOSECEF = require('./NavPOSECEF.js');
let NavSVINFO_SV = require('./NavSVINFO_SV.js');
let RxmRAWX = require('./RxmRAWX.js');
let CfgINF_Block = require('./CfgINF_Block.js');
let NavSOL = require('./NavSOL.js');
let EsfINS = require('./EsfINS.js');
let RxmSFRBX = require('./RxmSFRBX.js');
let NavSAT_SV = require('./NavSAT_SV.js');
let CfgUSB = require('./CfgUSB.js');
let RxmSVSI = require('./RxmSVSI.js');
let Ack = require('./Ack.js');
let CfgNAVX5 = require('./CfgNAVX5.js');
let CfgTMODE3 = require('./CfgTMODE3.js');
let NavDOP = require('./NavDOP.js');
let CfgNMEA7 = require('./CfgNMEA7.js');
let RxmSFRB = require('./RxmSFRB.js');
let RxmSVSI_SV = require('./RxmSVSI_SV.js');
let CfgGNSS_Block = require('./CfgGNSS_Block.js');
let NavPVT = require('./NavPVT.js');
let NavSVIN = require('./NavSVIN.js');
let NavATT = require('./NavATT.js');
let UpdSOS_Ack = require('./UpdSOS_Ack.js');
let CfgNMEA = require('./CfgNMEA.js');
let NavPOSLLH = require('./NavPOSLLH.js');
let EsfSTATUS_Sens = require('./EsfSTATUS_Sens.js');
let MonVER = require('./MonVER.js');
let CfgDGNSS = require('./CfgDGNSS.js');
let EsfMEAS = require('./EsfMEAS.js');
let NavDGPS_SV = require('./NavDGPS_SV.js');
let CfgNMEA6 = require('./CfgNMEA6.js');
let CfgANT = require('./CfgANT.js');
let NavSAT = require('./NavSAT.js');
let RxmRTCM = require('./RxmRTCM.js');
let NavSVINFO = require('./NavSVINFO.js');
let CfgRATE = require('./CfgRATE.js');
let RxmEPH = require('./RxmEPH.js');
let CfgRST = require('./CfgRST.js');
let EsfSTATUS = require('./EsfSTATUS.js');
let NavCLOCK = require('./NavCLOCK.js');
let CfgINF = require('./CfgINF.js');
let NavRELPOSNED = require('./NavRELPOSNED.js');

module.exports = {
  HnrPVT: HnrPVT,
  TimTM2: TimTM2,
  CfgPRT: CfgPRT,
  CfgCFG: CfgCFG,
  NavSBAS: NavSBAS,
  MonHW: MonHW,
  NavTIMEGPS: NavTIMEGPS,
  CfgDAT: CfgDAT,
  CfgSBAS: CfgSBAS,
  MgaGAL: MgaGAL,
  MonVER_Extension: MonVER_Extension,
  UpdSOS: UpdSOS,
  CfgGNSS: CfgGNSS,
  RxmRAW: RxmRAW,
  MonHW6: MonHW6,
  RxmRAWX_Meas: RxmRAWX_Meas,
  EsfRAW_Block: EsfRAW_Block,
  NavSTATUS: NavSTATUS,
  NavTIMEUTC: NavTIMEUTC,
  Inf: Inf,
  NavVELECEF: NavVELECEF,
  RxmRAW_SV: RxmRAW_SV,
  RxmALM: RxmALM,
  CfgMSG: CfgMSG,
  NavSBAS_SV: NavSBAS_SV,
  AidHUI: AidHUI,
  NavPVT7: NavPVT7,
  EsfRAW: EsfRAW,
  MonGNSS: MonGNSS,
  AidALM: AidALM,
  NavVELNED: NavVELNED,
  CfgNAV5: CfgNAV5,
  NavDGPS: NavDGPS,
  AidEPH: AidEPH,
  CfgHNR: CfgHNR,
  NavPOSECEF: NavPOSECEF,
  NavSVINFO_SV: NavSVINFO_SV,
  RxmRAWX: RxmRAWX,
  CfgINF_Block: CfgINF_Block,
  NavSOL: NavSOL,
  EsfINS: EsfINS,
  RxmSFRBX: RxmSFRBX,
  NavSAT_SV: NavSAT_SV,
  CfgUSB: CfgUSB,
  RxmSVSI: RxmSVSI,
  Ack: Ack,
  CfgNAVX5: CfgNAVX5,
  CfgTMODE3: CfgTMODE3,
  NavDOP: NavDOP,
  CfgNMEA7: CfgNMEA7,
  RxmSFRB: RxmSFRB,
  RxmSVSI_SV: RxmSVSI_SV,
  CfgGNSS_Block: CfgGNSS_Block,
  NavPVT: NavPVT,
  NavSVIN: NavSVIN,
  NavATT: NavATT,
  UpdSOS_Ack: UpdSOS_Ack,
  CfgNMEA: CfgNMEA,
  NavPOSLLH: NavPOSLLH,
  EsfSTATUS_Sens: EsfSTATUS_Sens,
  MonVER: MonVER,
  CfgDGNSS: CfgDGNSS,
  EsfMEAS: EsfMEAS,
  NavDGPS_SV: NavDGPS_SV,
  CfgNMEA6: CfgNMEA6,
  CfgANT: CfgANT,
  NavSAT: NavSAT,
  RxmRTCM: RxmRTCM,
  NavSVINFO: NavSVINFO,
  CfgRATE: CfgRATE,
  RxmEPH: RxmEPH,
  CfgRST: CfgRST,
  EsfSTATUS: EsfSTATUS,
  NavCLOCK: NavCLOCK,
  CfgINF: CfgINF,
  NavRELPOSNED: NavRELPOSNED,
};
