#include "ross.h"
#include "model.h"

tw_lptype model_lps[] =
{
  {
    (init_f) generator_init,
    (pre_run_f) generator_prerun,
    (event_f) generator_event_handler,
    (revent_f) generator_RC_event_handler,
    (commit_f) NULL,
    (final_f) generator_final,
    (map_f) model_map,
    sizeof(generator_state)
  },
  {
    (init_f) robot_init,
    (pre_run_f) robot_prerun,
    (event_f) robot_event_handler,
    (revent_f) robot_RC_event_handler,
    (commit_f) NULL,
    (final_f) robot_final,
    (map_f) model_map,
    sizeof(robot_state)
  },
  { 0 },
};


int total_robots = 5;
int total_generators = 1;

const tw_optdef model_opts[] = {
  TWOPT_GROUP("Model"),
  TWOPT_UINT("robots", total_robots, "Number of robots in simulation"),
  TWOPT_END()
};


void displayModelSettings()
{
  if (g_tw_mynode == 0)
  {
    for (int i = 0; i < 30; i++)
      printf("*");
    
    printf("\n");
    printf("Model Configuration:\n");
    printf("\t nnodes: %i\n", tw_nnodes());
    printf("\t g_tw_nlp: %llu\n", g_tw_nlp);
    printf("\t total_robots: %i\n", total_robots);
    printf("\t total_generators: %i\n\n", total_generators);

    for (int i = 0; i < 30; i++)
      printf("*");
    
    printf("\n");
  }
}


//for doxygen
#define mail_main main
int mail_main(int argc, char** argv, char **env)
{
  tw_opt_add(model_opts);
  tw_init(&argc, &argv);

  g_tw_nlp = total_robots + total_generators;  
  displayModelSettings();
  g_tw_lp_types = model_lps;
  g_tw_lp_typemap = lpTypeMapper;

  tw_define_lps(g_tw_nlp, sizeof(message));
  tw_lp_setup_types();
  tw_run();
  tw_end();
  
  return 0;
}