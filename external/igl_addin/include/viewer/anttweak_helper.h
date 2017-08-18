#pragma once

#define SET_TW_CALL_REGISTER(fname,twfname,t,cname)  static void TW_CALL twfname(const void *value, void *clientData){static_cast<cname*>(clientData)->fname(*static_cast<const t *>(value));}
#define GET_TW_CALL_REGISTER(fname,twfname,t,cname)  static void TW_CALL twfname(void *value, void *clientData){*static_cast<t *>(value) = static_cast<const cname*>(clientData)->fname();}

// The following are the auto variables register Macros
// You still need to define the get and set functions by yourself.
// fname: the name of the get and set functions are passed in.
// t: refers to the type of the variable, e.g. int, double, bool
// cname: the name of the class
#define DIALOG_SET_REGISTER(fname,t,cname)  static void TW_CALL dialog_##fname (const void *value, void *clientData){static_cast<cname*>(clientData)->fname(*static_cast<const t *>(value));}
#define DIALOG_GET_REGISTER(fname,t,cname)  static void TW_CALL dialog_##fname (void *value, void *clientData){*static_cast<t *>(value) = static_cast<const cname*>(clientData)->fname();}

#define DIALOG_OF(fname) dialog_##fname