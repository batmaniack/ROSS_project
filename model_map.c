#include "model.h"

tw_lpid lpTypeMapper(tw_lpid gid)
{
  if (gid == 0)
  { 
    return TYPE_GENERATOR;
  }
  else
  {
    return TYPE_ROBOT;
  } 
}

//Given an LP's GID (global ID)
//return the PE (aka node, MPI Rank)
tw_peid model_map(tw_lpid gid)
{
     return (tw_peid) gid / g_tw_nlp;
}