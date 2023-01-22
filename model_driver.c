#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <time.h>

#include "ross.h"
#include "model.h"

static int g_delivered_boxes = 0;

//////////////////////////////// DEBUG /////////////////////////////////////////
//#define MY_DEBUG

void delay(int number_of_milliseconds)
{
    clock_t start_time = clock();
    while (clock() < start_time + number_of_milliseconds)
        ;
}

#ifdef MY_DEBUG
#define debug_tw_event_send(e); \
        do { \
          printf("----- NEW EVENT -----\n");\
          printf("Line: %d\n", __LINE__);\
          printf("---------------------\n");\
          tw_event_send(e); \
        } while(0);
#else
#define debug_tw_event_send(e); tw_event_send(e);
#endif            

void print_robots_positions(tw_lp *lp)
{

  printf("\e[1;1H\e[2J");
  printf("\n====================== DEPOSITARY =======================\n");
  for (int i = 0; i <= MAX_ROW; ++i)
  {
    printf("|");
    for (int j = 0; j <= MAX_COL; ++j)
    {
      if (g_robots[i][j] == CARGO_ROBOT)
        printf("\033[0;31m   # \033[0;37m");
      else if (g_robots[i][j] == EMPTY_ROBOT)
        printf("\033[0;32m   # \033[0;37m");
      else if (g_robots[i][j] == NO_FUEL_ROBOT)
        printf("\033[0;33m   # \033[0;37m");
      else
      {
        if (g_depositary[i][j] > CHARGE_STATION)
          printf("\033[0;34m%4d \033[0;37m", g_depositary[i][j]);
        else
          printf("\033[0;37m%4d \033[0;37m", g_depositary[i][j]);
      }
    }
    printf("|\n");
  }
  printf("=========================================================\n");
  printf("Number of delivered boxes: %d\n", g_delivered_boxes);
  printf("Current simulation time: %lg", lp->pe->cur_event->recv_ts);
  fflush(stdout);
  delay(100000);
}

void print_robot_info(robot_state *s, message *in_msg, tw_lp *lp)
{
  int self = lp->gid;

  printf("\n====== ROBOT =====\n");
  printf("ID:   %d\n", self);
  printf("row:  %d\n", s->row);
  printf("col:  %d\n", s->col);
  printf("look: ");
  if (s->look_direction == D_UP)
    printf("UP\n");
  else if (s->look_direction == D_DOWN)
    printf("DOWN\n");
  else if (s->look_direction == D_LEFT)
    printf("LEFT\n");
  else if (s->look_direction == D_RIGHT)
    printf("RIGHT\n");
  else 
    printf("%d ERROR\n", s->look_direction);
  printf("fuel: %d\n", s->mileage_remain);
  printf("carg: %d\n\n", s->cargo_num);

  if (in_msg->type == MSGT_ROBOT_TO_ROBOT)
  {
    printf("Event type: ");
    if (in_msg->body.r_r.event_type == EVENTR_STAY)
      printf("STAY\n");
    else if (in_msg->body.r_r.event_type == EVENTR_MOVE)
      printf("MOVE\n");
    else if (in_msg->body.r_r.event_type == EVENTR_ROUND)
      printf("ROUND\n");
    else if (in_msg->body.r_r.event_type == EVENTR_CREATE_ROUTE_TO_GENERATOR)
      printf("CREATE_ROUTE_TO_GENERATOR\n");
    else if (in_msg->body.r_r.event_type == EVENTR_CREATE_ROUTE_TO_UNLOAD)
      printf("CREATE_ROUTE_TO_UNLOAD\n");
    else if (in_msg->body.r_r.event_type == EVENTR_CREATE_ROUTE_TO_CHARGE)
      printf("CREATE_ROUTE_TO_CHARGE\n");
    else
      printf("%d ERROR\n", in_msg->body.r_r.event_type);

    if (in_msg->body.r_r.event_type == EVENTR_MOVE)
    {
      printf("Direction: ");
      if (in_msg->body.r_r.direction == D_UP)
        printf("UP\n");
      else if (in_msg->body.r_r.direction == D_DOWN)
        printf("DOWN\n");
      else if (in_msg->body.r_r.direction == D_LEFT)
        printf("LEFT\n");
      else if (in_msg->body.r_r.direction == D_RIGHT)
        printf("RIGHT\n");
      else 
        printf("%d ERROR\n", in_msg->body.r_r.direction);
    }
    if (in_msg->body.r_r.event_type == EVENTR_ROUND)
    {
      printf("Angle: ");
      if (in_msg->body.r_r.angle == A_0)
        printf("0\n");
      if (in_msg->body.r_r.angle == A_90)
        printf("90\n");
      if (in_msg->body.r_r.angle == A_M90)
        printf("-90\n");
      if (in_msg->body.r_r.angle == A_180)
        printf("180\n");
    }
  }

  if (in_msg->type == MSGT_GENERATOR_TO_ROBOT)
  {
    printf("Received cargo: %d\n", in_msg->body.g_r.cargo_num);
  }

  printf("\n==================\n");
}



//////////////////////////////// ROBOT /////////////////////////////////////////

void robot_init(robot_state *s, tw_lp *lp)
{
  int self = lp->gid;

  s->mileage_remain = g_robot_max_mileage;
  s->cargo_num = NO_CARGO;

  // find place for robot
  int desired_row = 9;
  int desired_col = 5;

  while (g_robots[desired_row][desired_col] != 0)
  {
    if (desired_col != 1)
    {
      desired_col--;
      continue;
    }
    if (desired_row != 1)
    {
      desired_row--;
      continue;
    }
    printf("Too many robots\n");
    abort();
  }

  g_robots[desired_row][desired_col] = EMPTY_ROBOT;
  s->row = desired_row;
  s->col = desired_col;
  s->look_direction = D_UP;

  tw_event *e = tw_event_new(self, 1, lp);
  message *msg = tw_event_data(e);
  msg->type = MSGT_ROBOT_TO_ROBOT;
  msg->body.r_r.event_type = EVENTR_CREATE_ROUTE_TO_GENERATOR;
  msg->body.r_r.direction = D_STAY;
  msg->body.r_r.angle = A_0;
  debug_tw_event_send(e);
}

void robot_prerun(robot_state *s, tw_lp *lp)
{;}

void robot_round(robot_state *s, tw_lp *lp, Direction new_direction)
{
  assert(s->look_direction != new_direction);
  assert(new_direction != D_STAY);
  int self = lp->gid;
  tw_stime ts = 1;

  int round = new_direction - s->look_direction;
  if (round == 3 || round == -1)
  {
    ts = g_robot_calc_time;

    tw_event *e = tw_event_new(self, ts, lp);
    message *msg = tw_event_data(e);
    msg->type = MSGT_ROBOT_TO_ROBOT;
    msg->body.r_r.event_type = EVENTR_ROUND;
    msg->body.r_r.angle = A_M90;
    debug_tw_event_send(e);
    return;
  }
  if (round == 2 || round == -2)
  {
    ts = g_robot_calc_time;

    tw_event *e = tw_event_new(self, ts, lp);
    message *msg = tw_event_data(e);
    msg->type = MSGT_ROBOT_TO_ROBOT;
    msg->body.r_r.event_type = EVENTR_ROUND;
    msg->body.r_r.angle = A_180;
    debug_tw_event_send(e);
    return;
  }
  if (round == 1 || round == -3)
  {
    ts = g_robot_calc_time;

    tw_event *e = tw_event_new(self, ts, lp);
    message *msg = tw_event_data(e);
    msg->type = MSGT_ROBOT_TO_ROBOT;
    msg->body.r_r.event_type = EVENTR_ROUND;
    msg->body.r_r.angle = A_90;
    debug_tw_event_send(e);
    return;
  }

  printf("Unreachable route in robot_round\n");
  abort();
}

void robot_move(robot_state *s, tw_lp *lp, int new_row, int new_col)
{
  assert(s->col != new_col || s->row != new_row);
  int self = lp->gid;
  tw_stime ts = 1;

  if (new_row == 0 && new_col == 0)
  {
    assert(s->col != 0 && s->row != 0 && s->col != MAX_COL);

    if (s->row == 1)
    {
      if (s->col == 1 || s->col == MAX_COL-1)
      {
        if (s->look_direction == D_DOWN)
        {
          ts = g_robot_calc_time;
          
          tw_event *e = tw_event_new(self, ts, lp);
          message *msg = tw_event_data(e);
          msg->type = MSGT_ROBOT_TO_ROBOT;
          msg->body.r_r.event_type = EVENTR_MOVE;
          msg->body.r_r.direction = D_DOWN;
          debug_tw_event_send(e);
          return;
        }
        else
        {
          robot_round(s, lp, D_DOWN);
          return;
        }
      }

      if(s->col < 5)
      {
        if (s->look_direction == D_LEFT)
        {
        ts = g_robot_calc_time;
        
        tw_event *e = tw_event_new(self, ts, lp);
        message *msg = tw_event_data(e);
        msg->type = MSGT_ROBOT_TO_ROBOT;
        msg->body.r_r.event_type = EVENTR_MOVE;
        msg->body.r_r.direction = D_LEFT;
        debug_tw_event_send(e);
        return;
        }
        else
        {
          robot_round(s, lp, D_LEFT);
          return;
        }
      }
      else if(s->col > 5)
      {
        if (s->look_direction == D_RIGHT)
        {
        ts = g_robot_calc_time;
        
        tw_event *e = tw_event_new(self, ts, lp);
        message *msg = tw_event_data(e);
        msg->type = MSGT_ROBOT_TO_ROBOT;
        msg->body.r_r.event_type = EVENTR_MOVE;
        msg->body.r_r.direction = D_RIGHT;
        debug_tw_event_send(e);
        return;
        }
        else
        {
          robot_round(s, lp, D_RIGHT);
          return;
        }
      }
      else if (s->col == 5)
      {
        if (s->cargo_num % 2 == 0)
        {
          if (s->look_direction == D_RIGHT)
          {
            ts = g_robot_calc_time;
        
            tw_event *e = tw_event_new(self, ts, lp);
            message *msg = tw_event_data(e);
            msg->type = MSGT_ROBOT_TO_ROBOT;
            msg->body.r_r.event_type = EVENTR_MOVE;
            msg->body.r_r.direction = D_RIGHT;
            debug_tw_event_send(e);
            return;
          }
          else
          {
            robot_round(s, lp, D_RIGHT);
            return;
          }
        }
        else
        {
          if (s->look_direction == D_LEFT)
          {
            ts = g_robot_calc_time;
        
            tw_event *e = tw_event_new(self, ts, lp);
            message *msg = tw_event_data(e);
            msg->type = MSGT_ROBOT_TO_ROBOT;
            msg->body.r_r.event_type = EVENTR_MOVE;
            msg->body.r_r.direction = D_LEFT;
            debug_tw_event_send(e);
            return;
          }
          else
          {
            robot_round(s, lp, D_LEFT);
            return;
          }
        }
        
      }
    }

    if (s->row == MAX_ROW)
    {
      if (s->col == MAX_COL-1)
      {
        if (s->look_direction == D_LEFT)
        {
          ts = g_robot_calc_time;
          
          tw_event *e = tw_event_new(self, ts, lp);
          message *msg = tw_event_data(e);
          msg->type = MSGT_ROBOT_TO_ROBOT;
          msg->body.r_r.event_type = EVENTR_MOVE;
          msg->body.r_r.direction = D_LEFT;
          debug_tw_event_send(e);
          return;
        }
        else
        {
          robot_round(s, lp, D_LEFT);
          return;
        }
      }
    
      if (s->col == 1)
      {
        if (s->look_direction == D_RIGHT)
        {
          ts = g_robot_calc_time;
          
          tw_event *e = tw_event_new(self, ts, lp);
          message *msg = tw_event_data(e);
          msg->type = MSGT_ROBOT_TO_ROBOT;
          msg->body.r_r.event_type = EVENTR_MOVE;
          msg->body.r_r.direction = D_RIGHT;
          debug_tw_event_send(e);
          return;
        }
        else
        {
          robot_round(s, lp, D_RIGHT);
          return;
        }
      }

      if (s->col < 5)
      {
        if (s->look_direction == D_RIGHT)
        {
          ts = g_robot_calc_time;
        
          tw_event *e = tw_event_new(self, ts, lp);
          message *msg = tw_event_data(e);
          msg->type = MSGT_ROBOT_TO_ROBOT;
          msg->body.r_r.event_type = EVENTR_MOVE;
          msg->body.r_r.direction = D_RIGHT;
          debug_tw_event_send(e);
          return;
        }
        else
        {
          robot_round(s, lp, D_RIGHT);
          return;
        }
      }
      else if(s->col > 5)
      {
        if (s->look_direction == D_LEFT)
        {
          ts = g_robot_calc_time;
        
          tw_event *e = tw_event_new(self, ts, lp);
          message *msg = tw_event_data(e);
          msg->type = MSGT_ROBOT_TO_ROBOT;
          msg->body.r_r.event_type = EVENTR_MOVE;
          msg->body.r_r.direction = D_LEFT;
          debug_tw_event_send(e);
          return;
        }
        else
        {
          robot_round(s, lp, D_LEFT);
          return;
        }
      }
      else if(s->col == 5)
      {
        if (s->look_direction == D_UP)
        {
          ts = g_robot_calc_time;
        
          tw_event *e = tw_event_new(self, ts, lp);
          message *msg = tw_event_data(e);
          msg->type = MSGT_ROBOT_TO_ROBOT;
          msg->body.r_r.event_type = EVENTR_MOVE;
          msg->body.r_r.direction = D_UP;
          debug_tw_event_send(e);
          return;
        }
        else
        {
          robot_round(s, lp, D_UP);
          return;
        }
      }
    }
    
    if (s->col == MAX_COL-1 || s->col == 1)
    {
      if (s->look_direction == D_DOWN)
      {
        ts = g_robot_calc_time;
        
        tw_event *e = tw_event_new(self, ts, lp);
        message *msg = tw_event_data(e);
        msg->type = MSGT_ROBOT_TO_ROBOT;
        msg->body.r_r.event_type = EVENTR_MOVE;
        msg->body.r_r.direction = D_DOWN;
        debug_tw_event_send(e);
        return;
      }
      else
      {
        robot_round(s, lp, D_DOWN);
        return;
      }
    }

    if (s->col == 5)
    {
      if ((9 - (s->cargo_num + 1)/2) == s->row && s->cargo_num < 17)
      {
        if (s->cargo_num % 2 == 1 )
        {
          if (s->look_direction == D_LEFT)
          {
            ts = g_robot_calc_time;
        
            tw_event *e = tw_event_new(self, ts, lp);
            message *msg = tw_event_data(e);
            msg->type = MSGT_ROBOT_TO_ROBOT;
            msg->body.r_r.event_type = EVENTR_MOVE;
            msg->body.r_r.direction = D_LEFT;
            debug_tw_event_send(e);
            return;
          }
          else
          {
            robot_round(s, lp, D_LEFT);
            return;
          }
        }
        else 
        {
          if (s->look_direction == D_RIGHT)
          {
            ts = g_robot_calc_time;
        
            tw_event *e = tw_event_new(self, ts, lp);
            message *msg = tw_event_data(e);
            msg->type = MSGT_ROBOT_TO_ROBOT;
            msg->body.r_r.event_type = EVENTR_MOVE;
            msg->body.r_r.direction = D_RIGHT;
            debug_tw_event_send(e);
            return;
          }
          else
          {
            robot_round(s, lp, D_RIGHT);
            return;
          }
        }        
      }

      if (s->look_direction == D_UP)
      {
        ts = g_robot_calc_time;
        
        tw_event *e = tw_event_new(self, ts, lp);
        message *msg = tw_event_data(e);
        msg->type = MSGT_ROBOT_TO_ROBOT;
        msg->body.r_r.event_type = EVENTR_MOVE;
        msg->body.r_r.direction = D_UP;
        debug_tw_event_send(e);
        return;
      }
      else
      {
        robot_round(s, lp, D_UP);
        return;
      }
    }

    if (s->col < 5 && s->col > 1 && s->row > 1 && s->row < 7)
    {
      if (s->look_direction == D_LEFT)
          {
            ts = g_robot_calc_time;
        
            tw_event *e = tw_event_new(self, ts, lp);
            message *msg = tw_event_data(e);
            msg->type = MSGT_ROBOT_TO_ROBOT;
            msg->body.r_r.event_type = EVENTR_MOVE;
            msg->body.r_r.direction = D_LEFT;
            debug_tw_event_send(e);
            return;
          }
          else
          {
            robot_round(s, lp, D_LEFT);
            return;
          }
    }
    
    if (s->col > 5 && s->col < MAX_COL - 1 && s->row > 1 && s->row < 7)
    {
      if (s->look_direction == D_RIGHT)
          {
            ts = g_robot_calc_time;
        
            tw_event *e = tw_event_new(self, ts, lp);
            message *msg = tw_event_data(e);
            msg->type = MSGT_ROBOT_TO_ROBOT;
            msg->body.r_r.event_type = EVENTR_MOVE;
            msg->body.r_r.direction = D_RIGHT;
            debug_tw_event_send(e);
            return;
          }
          else
          {
            robot_round(s, lp, D_RIGHT);
            return;
          }
    }
  
  }

  if (new_col > MAX_COL || new_col < 0)
  {
    printf("Incorrect new_col\n");
    abort();
  }
  if (new_row > MAX_ROW || new_row < 0)
  {
    printf("Incorrect new_row\n");
    abort();
  }

  if (new_col < s->col)
  {
    if (s->look_direction == D_LEFT)
    {
      ts = g_robot_calc_time;
      
      tw_event *e = tw_event_new(self, ts, lp);
      message *msg = tw_event_data(e);
      msg->type = MSGT_ROBOT_TO_ROBOT;
      msg->body.r_r.event_type = EVENTR_MOVE;
      msg->body.r_r.direction = D_LEFT;
      debug_tw_event_send(e);
      return;
    }
    else
    {
      robot_round(s, lp, D_LEFT);
      return;
    }
  }
  if (new_col > s->col)
  {
    if (s->look_direction == D_RIGHT)
    {
      ts = g_robot_calc_time;
      
      tw_event *e = tw_event_new(self, ts, lp);
      message *msg = tw_event_data(e);
      msg->type = MSGT_ROBOT_TO_ROBOT;
      msg->body.r_r.event_type = EVENTR_MOVE;
      msg->body.r_r.direction = D_RIGHT;
      debug_tw_event_send(e);
      return;
    }
    else
    {
      robot_round(s, lp, D_RIGHT);
      return;
    }
  }

  if (new_row < s->row)
  {
    if (s->look_direction == D_UP)
    {
      ts = g_robot_calc_time;
      
      tw_event *e = tw_event_new(self, ts, lp);
      message *msg = tw_event_data(e);
      msg->type = MSGT_ROBOT_TO_ROBOT;
      msg->body.r_r.event_type = EVENTR_MOVE;
      msg->body.r_r.direction = D_UP;
      debug_tw_event_send(e);
      return;
    }
    else
    {
      robot_round(s, lp, D_UP);
      return;
    }
  }
  if (new_row > s->row)
  {
    if (s->look_direction == D_DOWN)
    {
      ts = g_robot_calc_time;
      
      tw_event *e = tw_event_new(self, ts, lp);
      message *msg = tw_event_data(e);
      msg->type = MSGT_ROBOT_TO_ROBOT;
      msg->body.r_r.event_type = EVENTR_MOVE;
      msg->body.r_r.direction = D_DOWN;
      debug_tw_event_send(e);
      return;
    }
    else
    {
      robot_round(s, lp, D_DOWN);
      return;
    }
  }

  printf("Unreachable route in robot_move\n");
  abort();
}

void robot_event_handler(robot_state *s, tw_bf *bf, message *in_msg, tw_lp *lp)
{
  int self = lp->gid;
  tw_stime ts = 1;

  if (in_msg->type == MSGT_ROBOT_TO_ROBOT)
  {
    #ifdef MY_DEBUG
      print_robot_info(s, in_msg, lp);
    #endif
    
    switch (in_msg->body.r_r.event_type)
    {
      ///////////////////////////// STAY ///////////////////////////////////////
      case EVENTR_STAY: 
        if (s->cargo_num == NO_CARGO)
        {
          ts = g_robot_calc_time;

          tw_event *e = tw_event_new(self, ts, lp);
          message *msg = tw_event_data(e);
          msg->type = MSGT_ROBOT_TO_ROBOT;
          msg->body.r_r.event_type = EVENTR_CREATE_ROUTE_TO_GENERATOR;
          debug_tw_event_send(e);
          break;
        }
        if (s->mileage_remain < g_robot_min_mileage)
        {
          ts = g_robot_calc_time;

          tw_event *e = tw_event_new(self, ts, lp);
          message *msg = tw_event_data(e);
          msg->type = MSGT_ROBOT_TO_ROBOT;
          msg->body.r_r.event_type = EVENTR_CREATE_ROUTE_TO_CHARGE;
          msg->body.r_r.direction = D_STAY;
          debug_tw_event_send(e);
          break;
        }
        if (s->cargo_num != NO_CARGO)
        {
          ts = g_robot_calc_time;

          tw_event *e = tw_event_new(self, ts, lp);
          message *msg = tw_event_data(e);
          msg->type = MSGT_ROBOT_TO_ROBOT;
          msg->body.r_r.event_type = EVENTR_CREATE_ROUTE_TO_UNLOAD;
          debug_tw_event_send(e);
          break;
        }
        printf("Unreachable route in EVENTR_STAY");
        abort();
        break;
      ///////////////////////////// MOVE ///////////////////////////////////////
      case EVENTR_MOVE:
        if (s->mileage_remain == 0)
        {
          g_robots[s->row][s->col] = NO_FUEL_ROBOT;

          ts = g_robot_wait_time;
          tw_event *e = tw_event_new(self, ts, lp);
          message *msg = tw_event_data(e);
          msg->type = MSGT_ROBOT_TO_ROBOT;
          msg->body.r_r.event_type = EVENTR_STAY;
          debug_tw_event_send(e);
          print_robots_positions(lp);
          break;
        }
        switch (in_msg->body.r_r.direction)
        {
          case D_UP:
            if (s->row == 0)
            {
              printf("Can not do D_UP");
              abort();
            }
            if (g_robots[s->row-1][s->col] != NO_ROBOT)
            {
              ts = g_robot_calc_time;

              tw_event *e = tw_event_new(self, ts, lp);
              message *msg = tw_event_data(e);
              msg->type = MSGT_ROBOT_TO_ROBOT;
              msg->body.r_r.event_type = EVENTR_MOVE;
              msg->body.r_r.direction = D_UP;
              debug_tw_event_send(e);
              break;
            }
            else
            {
              ts = g_robot_move_time;

              s->row = s->row - 1;
              s->mileage_remain -= 1;

              if (s->cargo_num == NO_CARGO)
                g_robots[s->row][s->col] = EMPTY_ROBOT;
              else
                g_robots[s->row][s->col] = CARGO_ROBOT;
              
              g_robots[s->row + 1][s->col] = NO_ROBOT;

              tw_event *e = tw_event_new(self, ts, lp);
              message *msg = tw_event_data(e);
              msg->type = MSGT_ROBOT_TO_ROBOT;
              msg->body.r_r.event_type = EVENTR_STAY;
              debug_tw_event_send(e);
              print_robots_positions(lp);
              break;
            }
            break;
          case D_DOWN:
            if (s->row == MAX_ROW)
            {
              printf("Can not do D_DOWN");
              abort();
            }
            if (g_robots[s->row+1][s->col] != NO_ROBOT)
            {
              ts = g_robot_wait_time;

              tw_event *e = tw_event_new(self, ts, lp);
              message *msg = tw_event_data(e);
              msg->type = MSGT_ROBOT_TO_ROBOT;
              msg->body.r_r.event_type = EVENTR_MOVE;
              msg->body.r_r.direction = D_DOWN;
              debug_tw_event_send(e);
              break;
            }
            else
            {
              ts = g_robot_calc_time;

              s->row = s->row + 1;
              s->mileage_remain -= 1;
              if (s->cargo_num == NO_CARGO)
                g_robots[s->row][s->col] = EMPTY_ROBOT;
              else
                g_robots[s->row][s->col] = CARGO_ROBOT;
              
              g_robots[s->row - 1][s->col] = NO_ROBOT;

              tw_event *e = tw_event_new(self, ts, lp);
              message *msg = tw_event_data(e);
              msg->type = MSGT_ROBOT_TO_ROBOT;
              msg->body.r_r.event_type = EVENTR_STAY;
              debug_tw_event_send(e);
              print_robots_positions(lp);
              break;
            }
            break;
          case D_LEFT:
            if (s->col == 0)
            {
              printf("Can not do D_LEFT");
              abort();
            }
            if (g_robots[s->row][s->col-1] != NO_ROBOT)
            {
              ts = g_robot_calc_time;

              tw_event *e = tw_event_new(self, ts, lp);
              message *msg = tw_event_data(e);
              msg->type = MSGT_ROBOT_TO_ROBOT;
              msg->body.r_r.event_type = EVENTR_MOVE;
              msg->body.r_r.direction = D_LEFT;
              debug_tw_event_send(e);
              break;
            }
            else
            {
              ts = g_robot_move_time;

              s->col = s->col - 1;
              s->mileage_remain -= 1;
              if (s->cargo_num == NO_CARGO)
                g_robots[s->row][s->col] = EMPTY_ROBOT;
              else
                g_robots[s->row][s->col] = CARGO_ROBOT;
              
              g_robots[s->row][s->col+1] = NO_ROBOT;

              tw_event *e = tw_event_new(self, ts, lp);
              message *msg = tw_event_data(e);
              msg->type = MSGT_ROBOT_TO_ROBOT;
              msg->body.r_r.event_type = EVENTR_STAY;
              debug_tw_event_send(e);
              print_robots_positions(lp);
              break;
            }
            break;
          case D_RIGHT:
            if (s->col == MAX_COL)
            {
              printf("Can not do D_RIGHT");
              abort();
            }
            if (g_robots[s->row][s->col+1] != NO_ROBOT)
            {
              ts = g_robot_calc_time;

              tw_event *e = tw_event_new(self, ts, lp);
              message *msg = tw_event_data(e);
              msg->type = MSGT_ROBOT_TO_ROBOT;
              msg->body.r_r.event_type = EVENTR_MOVE;
              msg->body.r_r.direction = D_RIGHT;
              debug_tw_event_send(e);
              break;
            }
            else
            {
              ts = g_robot_move_time;

              s->col = s->col + 1;
              s->mileage_remain -= 1;
              if (s->cargo_num == NO_CARGO)
                g_robots[s->row][s->col] = EMPTY_ROBOT;
              else
                g_robots[s->row][s->col] = CARGO_ROBOT;
              
              g_robots[s->row][s->col-1] = NO_ROBOT;

              tw_event *e = tw_event_new(self, ts, lp);
              message *msg = tw_event_data(e);
              msg->type = MSGT_ROBOT_TO_ROBOT;
              msg->body.r_r.event_type = EVENTR_STAY;
              debug_tw_event_send(e);
              print_robots_positions(lp);
              break;
            }
            break;
          default:
            printf("Unreachable route in EVENTR_MOVE\n");
            abort();
        }
        break;
      
      /////////////////////////////// ROUND ////////////////////////////////////
      case EVENTR_ROUND:
        switch (in_msg->body.r_r.angle)
        {
          case A_90:
            s->look_direction = (s->look_direction + 1) % D_STAY;
            ts = g_robot_round_time;
            break;
          case A_M90:
            s->look_direction = (D_STAY + s->look_direction - 1) % D_STAY;
            ts = g_robot_round_time;
            break;
          case A_180:
            s->look_direction = (s->look_direction + 2) % D_STAY;
            ts = g_robot_round_time * 2;
            break;
        }
        tw_event *e = tw_event_new(self, ts, lp);
        message *msg = tw_event_data(e);
        msg->type = MSGT_ROBOT_TO_ROBOT;
        msg->body.r_r.event_type = EVENTR_STAY;
        debug_tw_event_send(e);
        break;
      /////////////////////////// TO GENERATOR /////////////////////////////////
      case EVENTR_CREATE_ROUTE_TO_GENERATOR: 
        // if we are ON generator
        if (g_depositary[s->row][s->col] == GENERATOR_STATION)
        {
          ts = g_robot_calc_time;
          tw_event *e = tw_event_new(0, ts, lp);
          message *msg = tw_event_data(e);
          msg->type = MSGT_ROBOT_TO_GENERATOR;
          msg->body.r_g.robot_id = self;
          debug_tw_event_send(e);
          break;
        }

        // if we are ON edge
        if (s->col == 0)
        {
          robot_move(s, lp, s->row, s->col+1);
          break;
        }
        if (s->row == 0)
        {
          robot_move(s, lp, s->row+1, s->col);
          break;
        }
        if (s->col == MAX_COL)
        {
          robot_move(s, lp, s->row, s->col-1);
          break;
        }

        robot_move(s, lp, 0, 0);
        break;

      ///////////////////////////// TO UNLOAD //////////////////////////////////
      case EVENTR_CREATE_ROUTE_TO_UNLOAD:

        // if we are ON onload
        if (g_depositary[s->row][s->col] == s->cargo_num)
        {
          s->cargo_num = NO_CARGO;
          g_delivered_boxes++;

          ts = g_robot_unload_time;
          tw_event *e = tw_event_new(self, ts, lp);
          message *msg = tw_event_data(e);
          msg->type = MSGT_ROBOT_TO_ROBOT;
          msg->body.r_r.event_type = EVENTR_STAY;
          debug_tw_event_send(e);
          break;
        }


        // if we are ON edge
        if (s->col == 0)
        {
          robot_move(s, lp, s->row, s->col+1);
          break;
        }
        if (s->row == 0)
        {
          robot_move(s, lp, s->row+1, s->col);
          break;
        }
        if (s->col == MAX_COL)
        {
          robot_move(s, lp, s->row, s->col-1);
          break;
        }

        // if we are near unload
        if (s->col == 1 && 
            g_depositary[s->row][s->col-1] == s->cargo_num &&
            g_robots[s->row][s->col-1] == NO_ROBOT)
        {
          robot_move(s, lp, s->row, s->col-1);
          break;
        }

        if (s->col == MAX_COL-1 && 
            g_depositary[s->row][s->col+1] == s->cargo_num &&
            g_robots[s->row][s->col+1] == NO_ROBOT)
        {
          robot_move(s, lp, s->row, s->col+1);
          break;
        }

        if (s->row == 1 && 
            g_depositary[s->row-1][s->col] == s->cargo_num &&
            g_robots[s->row-1][s->col] == NO_ROBOT)
        {
          robot_move(s, lp, s->row-1, s->col);
          break;
        }

        robot_move(s, lp, 0, 0);
        break;

      ///////////////////////////// TO CHARGE //////////////////////////////////
      case EVENTR_CREATE_ROUTE_TO_CHARGE: 

        // if we are ON charge
        if (g_depositary[s->row][s->col] == CHARGE_STATION)
        {
          s->mileage_remain = g_robot_max_mileage;

          ts = g_robot_charge_time;
          tw_event *e = tw_event_new(self, ts, lp);
          message *msg = tw_event_data(e);
          msg->type = MSGT_ROBOT_TO_ROBOT;
          msg->body.r_r.event_type = EVENTR_STAY;
          debug_tw_event_send(e);
          break;
        }

        // if we are ON edge
        if (s->col == 0)
        {
          robot_move(s, lp, s->row, s->col+1);
          break;
        }
        if (s->row == 0)
        {
          robot_move(s, lp, s->row+1, s->col);
          break;
        }
        if (s->col == MAX_COL)
        {
          robot_move(s, lp, s->row, s->col-1);
          break;
        }

        // if we are near charge
        if (s->col == 1 && 
            g_depositary[s->row][s->col-1] == CHARGE_STATION &&
            g_robots[s->row][s->col-1] == NO_ROBOT)
        {
          robot_move(s, lp, s->row, s->col-1);
          break;
        }

        if (s->col == MAX_COL-1 && 
            g_depositary[s->row][s->col+1] == CHARGE_STATION &&
            g_robots[s->row][s->col+1] == NO_ROBOT)
        {
          robot_move(s, lp, s->row, s->col+1);
          break;
        }

        robot_move(s, lp, 0, 0);
        break;
      
      default:
        printf("Unknown robot event\n");
        abort();
    }
  }
  else if (in_msg->type == MSGT_GENERATOR_TO_ROBOT)
  {
    s->cargo_num = in_msg->body.g_r.cargo_num;

    if ( s->row < 5)
    robot_move(s, lp, s->row + 1, s->col);
    else 
    robot_move(s, lp, s->row - 1, s->col);
  }
  else
  {
    printf("Unknown message type!\n");
    abort();
  }
}

void robot_RC_event_handler(robot_state *s, tw_bf *bf, message *in_msg, tw_lp *lp)
{
  printf("I have not implemented robot_RC_event_handler\n");
  abort();
}

void robot_final(robot_state *s, tw_lp *lp)
{
  int self = lp->gid;
  printf("Robot: %d, mileage_remain: %d, cargo_num: %d\n", 
    self, s->mileage_remain, s->cargo_num);
}

////////////////////////////// GENERATOR ///////////////////////////////////////

void generator_init (generator_state *s, tw_lp *lp)
{
  int self = lp->gid;

  s->current_cargo = 0;

  FILE* fp = fopen("distr.txt", "r");
  if (fp == NULL) 
  {
    printf("Can not find distr.txt\n");
    abort();
  }

  int cargo = 0;
  int hour = 0;
  int minute = 0;
  int second = 0;
  int i = 0;
  while (fscanf(fp, "%02d:%02d:%02d %d", &hour, &minute, &second, &cargo) == 4)
  {
    if (i > MAX_CARGO_NUMBER)
      break;

    s->cargo_timetable[i][0] = second + minute*60 + hour*60*60;
    s->cargo_timetable[i][1] = 
      cargo % (CARGO_PLACE_MAX_NUM - CARGO_PLACE_MIN_NUM) + CARGO_PLACE_MIN_NUM;
    i++;
  }
  
  printf("Generator created with gid == %d\n", self);
  for (int i = 0; i < 10; ++i)
  {
    printf("time: %4d; num: %2d\n", 
      s->cargo_timetable[i][0], s->cargo_timetable[i][1]);
  }
  printf(" ...\n");
}

void generator_prerun (generator_state *s, tw_lp *lp)
{;}


void generator_event_handler(generator_state *s, tw_bf *bf, message *in_msg, tw_lp *lp)
{
  int self = lp->gid;
  tw_stime ts = g_robot_load_time;
  int ret_cargo = 0;

  if (s->current_cargo + 1 == MAX_CARGO_NUMBER)
    ts = g_robot_calc_time;
  else
    ret_cargo = s->cargo_timetable[s->current_cargo++][1];

  tw_event *e = tw_event_new(in_msg->body.r_g.robot_id, ts, lp);
  message *msg = tw_event_data(e);
  msg->type = MSGT_GENERATOR_TO_ROBOT;
  msg->body.g_r.cargo_num = ret_cargo;
  debug_tw_event_send(e);
}

void generator_RC_event_handler(generator_state *s, tw_bf *bf, message *in_msg, tw_lp *lp)
{
  printf("I have not implemented generator_RC_event_handler\n");
  abort();
}

void generator_final(generator_state *s, tw_lp *lp)
{
  int self = lp->gid;
  printf("Generator: %d, last cargo:%d\n", self, s->current_cargo);
}