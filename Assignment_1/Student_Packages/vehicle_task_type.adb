-- Suggestions for packages which might be useful:

with Ada.Real_Time;              use Ada.Real_Time;
with Real_Type;                  use Real_Type;
with Exceptions;                 use Exceptions;
--  with Generic_Sliding_Statistics;
with Vectors_3D;                 use Vectors_3D;
with Vehicle_Interface;          use Vehicle_Interface;
with Vehicle_Message_Type;       use Vehicle_Message_Type;
with Swarm_Structures;           use Swarm_Structures;
with Swarm_Structures_Base;      use Swarm_Structures_Base;
with Swarm_Configuration;        use Swarm_Configuration;

package body Vehicle_Task_Type is

   task body Vehicle_Task is

      Vehicle_No : Positive;
      -- You will want to take the pragma out, once you use the "Vehicle_No"

      -- the message of the current vehicle
      message : Inter_Vehicle_Messages;

      -- if found 2 or more globes this boolean variable will be True
      dual_globes_detected : Boolean := False;

      -- the radius for the orbit
      max_anchor_distance :  Long_Float;

      -- time stamp for init
      init_clock : constant Time := Clock;

   begin

      -- You need to react to this call and provide your task_id.
      -- You can e.g. employ the assigned vehicle number (Vehicle_No)
      -- in communications with other vehicles.

      accept Identify (Set_Vehicle_No : Positive; Local_Task_Id : out Task_Id) do
         Vehicle_No     := Set_Vehicle_No;
         Local_Task_Id  := Current_Task;
      end Identify;

      -- Replace the rest of this task with your own code.
      -- Maybe synchronizing on an external event clock like "Wait_For_Next_Physics_Update",
      -- yet you can synchronize on e.g. the real-time clock as well.

      -- Without control this vehicle will go for its natural swarming instinct.

      -------------------------------------------------------------------
      -- The if else statement is to determine the radius of the orbit --
      -------------------------------------------------------------------

      select

         Flight_Termination.Stop;

      then abort

         Outer_task_loop : loop

            Wait_For_Next_Physics_Update;

            -- Your vehicle should respond to the world here: sense, listen, talk, act?

            declare
               -- Detecting the globes around the vehicle
               allglobes : constant Energy_Globes := Energy_Globes_Around;
               -- messages for communication between vehicles(tasks) in a distributed way
               this_message : Inter_Vehicle_Messages := ((1.0, 0.0, 0.0), init_clock, 0, 0, 0, (0.0, 0.0, 0.0));
               another_message : Inter_Vehicle_Messages := ((1.0, 0.0, 0.0), init_clock, 0, 0, 0, (0.0, 0.0, 0.0));
               -- time variable help to show the time difference
               current_time : constant Time := Clock;

               -- present the current energy of a vehicle, range from 0 to 1
               current_energy : Vehicle_Charges;

               -- A position for the vehicle, which is used for calculate the destination
               waiting_position : Positions;

               -- Parameters to distinguish whether in single_globe problem or in dual_globe_problem
               anchor_vehicle_number : Positive;

               -- The offset in 3D space for the destination to consist a orbit of a group of satellites.
               -- Here are only three orbits, also we could make more orbits to imporve the space efficiency.
               orbit_matrix : constant array (0 .. 2, 0 .. 7) of Vector_3D :=
                 (
                    ((max_anchor_distance, 0.0, -0.1), (0.7 * max_anchor_distance, 0.7 * max_anchor_distance, -0.1), (0.0, max_anchor_distance, -0.1), (-0.7 * max_anchor_distance, 0.7 * max_anchor_distance, -0.1), (-max_anchor_distance, 0.0, -0.1), (-0.7 * max_anchor_distance, -0.7 * max_anchor_distance, -0.1), (0.0, -max_anchor_distance, -0.1), (0.7 * max_anchor_distance, -0.7 * max_anchor_distance, -0.1)),
                  ((max_anchor_distance, 0.0, 0.0), (0.7 * max_anchor_distance, 0.7 * max_anchor_distance, 0.0), (0.0, max_anchor_distance, 0.0), (-0.7 * max_anchor_distance, 0.7 * max_anchor_distance, 0.0), (-max_anchor_distance, 0.0, 0.0), (-0.7 * max_anchor_distance, -0.7 * max_anchor_distance, 0.0), (0.0, -max_anchor_distance, 0.0), (0.7 * max_anchor_distance, -0.7 * max_anchor_distance, 0.0)),
                  ((max_anchor_distance, 0.0, 0.1), (0.7 * max_anchor_distance, 0.7 * max_anchor_distance, 0.1), (0.0, max_anchor_distance, 0.1), (-0.7 * max_anchor_distance, 0.7 * max_anchor_distance, 0.1), (-max_anchor_distance, 0.0, 0.1), (-0.7 * max_anchor_distance, -0.7 * max_anchor_distance, 0.1), (0.0, -max_anchor_distance, 0.1), (0.7 * max_anchor_distance, -0.7 * max_anchor_distance, 0.1))
                 );

               -- Single globe mode message
               single_message : Inter_Vehicle_Messages;
               single : Boolean := False;

               -- Dual globes mode messages, including same group message and different group message.
               same_group_message : Inter_Vehicle_Messages := ((1.0, 0.0, 0.0), init_clock, 0, 0, 0, (0.0, 0.0, 0.0));
               diff_group_message : Inter_Vehicle_Messages := ((1.0, 0.0, 0.0), init_clock, 0, 0, 0, (0.0, 0.0, 0.0));
               same_group : Boolean := False;
               diff_group : Boolean := False;

            begin
               -----------
               -- Step1 -- vehicles try to find the energy globes and communicate with each other
               -----------

               pragma Warnings (Off, Swarm_Configuration);

               if dual_globes_detected then

                  if Swarm_Configuration.Initial_No_of_Elements <= 20 then
                     max_anchor_distance := 0.03;
                  elsif Swarm_Configuration.Initial_No_of_Elements <= 40 then
                     max_anchor_distance := 0.06;
                  elsif Swarm_Configuration.Initial_No_of_Elements <= 80 then
                     max_anchor_distance := 0.1;
                  elsif Swarm_Configuration.Initial_No_of_Elements <= 120 then
                     max_anchor_distance := 0.15;
                  elsif Swarm_Configuration.Initial_No_of_Elements <= 160 then
                     max_anchor_distance := 0.2;
                  elsif Swarm_Configuration.Initial_No_of_Elements <= 200 then
                     max_anchor_distance := 0.22;
                  elsif Swarm_Configuration.Initial_No_of_Elements <= 240 then
                     max_anchor_distance := 0.25;
                  elsif Swarm_Configuration.Initial_No_of_Elements <= 280 then
                     max_anchor_distance := 0.28;
                  else
                     max_anchor_distance := 0.3;
                  end if;

               else
                  if Swarm_Configuration.Initial_No_of_Elements <= 10 then -- holds in 151
                     max_anchor_distance := 0.03;
                  elsif Swarm_Configuration.Initial_No_of_Elements <= 20 then
                     max_anchor_distance := 0.06;
                  elsif Swarm_Configuration.Initial_No_of_Elements <= 40 then
                     max_anchor_distance := 0.1;
                  elsif Swarm_Configuration.Initial_No_of_Elements <= 60 then
                     max_anchor_distance := 0.15;
                  elsif Swarm_Configuration.Initial_No_of_Elements <= 80 then
                     max_anchor_distance := 0.2;
                  elsif Swarm_Configuration.Initial_No_of_Elements <= 120 then
                     max_anchor_distance := 0.25;
                  elsif Swarm_Configuration.Initial_No_of_Elements <= 140 then
                     max_anchor_distance := 0.28;
                  elsif Swarm_Configuration.Initial_No_of_Elements > 140 then
                     max_anchor_distance := 0.3;
                  end if;
               end if;

               pragma Warnings (On, Swarm_Configuration);


               if allglobes'Length > 0 then  -- vehicles who can detecte some globes

                  for globes of allglobes loop
                     this_message.globe := globes.Position;
                     this_message.update_time :=  current_time;
                     this_message.source_vehicle := Vehicle_No;
                     this_message.sending_vehicle := Vehicle_No;

                     if Vehicle_No = 1 then
                        this_message.v1_position := Position;
                     end if;

                     Send (this_message);
                  end loop;
               else -- vehicles who cannot detecte some globes

                  while Messages_Waiting loop  -- wait for message and determine which one is the best
                     Receive (another_message);

                     if another_message.update_time < message.update_time and then abs (another_message.v1_position - (0.0, 0.0, 0.0)) > 0.001 and then another_message.source_vehicle = 0 and then another_message.sending_vehicle = 0 then
                        null; -- some useless messages
                     else

                        -- for detecting whether there are more than one globe by calculating the distance in a short time interval
                        if dual_globes_detected = False then
                           if Real (To_Duration (another_message.update_time - message.update_time)) < 0.03  then
                              if abs (another_message.globe - message.globe) > 0.15 then
                                 dual_globes_detected := True;
                              end if;
                           end if;
                        end if;

                        -- Here comes into the dual mode
                        if dual_globes_detected then

                           if Vehicle_No = 1 then -- vehicle 1 is very specilal, it ought to chase the globe all the time to be the guider
                              if abs (message.globe - Position) <= 0.004 then
                                 -- this guider need to ignore other message when it already get close to the target globe
                                 if abs (another_message.globe - Position) > abs (message.globe - Position) then
                                    null;
                                 else
                                    if (another_message.sending_vehicle mod 2) = (Vehicle_No mod 2) then
                                       diff_group := False;
                                       if same_group then
                                          if another_message.update_time > same_group_message.update_time then
                                             same_group_message.globe := another_message.globe;
                                             same_group_message.update_time := another_message.update_time;
                                             same_group_message.source_vehicle := another_message.source_vehicle;
                                             same_group_message.v1_position := Position;
                                          end if;
                                       else
                                          same_group := True;
                                          same_group_message.globe := another_message.globe;
                                          same_group_message.update_time := another_message.update_time;
                                          same_group_message.source_vehicle := another_message.source_vehicle;
                                          same_group_message.v1_position := Position;
                                       end if;
                                    else
                                       if same_group then
                                          null;
                                       else
                                          if diff_group then
                                             if another_message.update_time > same_group_message.update_time then
                                                diff_group_message.globe := another_message.globe;
                                                diff_group_message.update_time := another_message.update_time;
                                                diff_group_message.source_vehicle := another_message.source_vehicle;
                                                diff_group_message.v1_position := Position;
                                             end if;
                                          else
                                             diff_group := True;
                                             diff_group_message.globe := another_message.globe;
                                             diff_group_message.update_time := another_message.update_time;
                                             diff_group_message.source_vehicle := another_message.source_vehicle;
                                             diff_group_message.v1_position := Position;
                                          end if;
                                       end if;
                                    end if;
                                 end if;
                              else
                                 if (another_message.sending_vehicle mod 2) = (Vehicle_No mod 2) then
                                    diff_group := False;
                                    if same_group then
                                       if another_message.update_time > same_group_message.update_time then
                                          same_group_message.globe := another_message.globe;
                                          same_group_message.update_time := another_message.update_time;
                                          same_group_message.source_vehicle := another_message.source_vehicle;
                                          same_group_message.v1_position := Position;
                                       end if;
                                    else
                                       same_group := True;
                                       same_group_message.globe := another_message.globe;
                                       same_group_message.update_time := another_message.update_time;
                                       same_group_message.source_vehicle := another_message.source_vehicle;
                                       same_group_message.v1_position := Position;
                                    end if;
                                 else
                                    if same_group then
                                       null;
                                    else
                                       if diff_group then
                                          if another_message.update_time > same_group_message.update_time then
                                             diff_group_message.globe := another_message.globe;
                                             diff_group_message.update_time := another_message.update_time;
                                             diff_group_message.source_vehicle := another_message.source_vehicle;
                                             diff_group_message.v1_position := Position;
                                          end if;
                                       else
                                          diff_group := True;
                                          diff_group_message.globe := another_message.globe;
                                          diff_group_message.update_time := another_message.update_time;
                                          diff_group_message.source_vehicle := another_message.source_vehicle;
                                          diff_group_message.v1_position := Position;
                                       end if;
                                    end if;
                                 end if;
                              end if;

                           elsif Vehicle_No = 2 then
                              -- vehicle 2 is also very specilal, it ought to chase the globe all the time to be the guider
                              -- but it should not be chase the same globe with vehicle 1

                              if abs (another_message.globe - message.v1_position) < 0.01 then
                                 null;

                              else

                                 if (another_message.sending_vehicle mod 2) = (Vehicle_No mod 2) then
                                    diff_group := False;
                                    if same_group then
                                       if another_message.update_time > same_group_message.update_time then
                                          same_group_message.globe := another_message.globe;
                                          same_group_message.update_time := another_message.update_time;
                                          same_group_message.source_vehicle := another_message.source_vehicle;
                                          same_group_message.v1_position := another_message.v1_position;
                                       end if;
                                    else
                                       same_group := True;
                                       same_group_message.globe := another_message.globe;
                                       same_group_message.update_time := another_message.update_time;
                                       same_group_message.source_vehicle := another_message.source_vehicle;
                                       same_group_message.v1_position := another_message.v1_position;
                                    end if;
                                 else
                                    if same_group then
                                       null;
                                    else
                                       if diff_group then
                                          if another_message.update_time > same_group_message.update_time then
                                             diff_group_message.globe := another_message.globe;
                                             diff_group_message.update_time := another_message.update_time;
                                             diff_group_message.source_vehicle := another_message.source_vehicle;
                                             diff_group_message.v1_position := another_message.v1_position;
                                          end if;
                                       else
                                          diff_group := True;
                                          diff_group_message.globe := another_message.globe;
                                          diff_group_message.update_time := another_message.update_time;
                                          diff_group_message.source_vehicle := another_message.source_vehicle;
                                          diff_group_message.v1_position := another_message.v1_position;
                                       end if;
                                    end if;
                                 end if;
                              end if;

                           else -- Here comes into the single mode
                              if (another_message.sending_vehicle mod 2) = (Vehicle_No mod 2) then
                                 diff_group := False;
                                 if same_group then
                                    if another_message.update_time > same_group_message.update_time then
                                       same_group_message.globe := another_message.globe;
                                       same_group_message.update_time := another_message.update_time;
                                       same_group_message.source_vehicle := another_message.source_vehicle;
                                       same_group_message.v1_position := another_message.v1_position;
                                    end if;
                                 else
                                    same_group := True;
                                    same_group_message.globe := another_message.globe;
                                    same_group_message.update_time := another_message.update_time;
                                    same_group_message.source_vehicle := another_message.source_vehicle;
                                    same_group_message.v1_position := another_message.v1_position;
                                 end if;
                              else
                                 if same_group then
                                    null;
                                 else
                                    if diff_group then
                                       if another_message.update_time > same_group_message.update_time then
                                          diff_group_message.globe := another_message.globe;
                                          diff_group_message.update_time := another_message.update_time;
                                          diff_group_message.source_vehicle := another_message.source_vehicle;
                                          diff_group_message.v1_position := another_message.v1_position;
                                       end if;
                                    else
                                       diff_group := True;
                                       diff_group_message.globe := another_message.globe;
                                       diff_group_message.update_time := another_message.update_time;
                                       diff_group_message.source_vehicle := another_message.source_vehicle;
                                       diff_group_message.v1_position := another_message.v1_position;
                                    end if;
                                 end if;
                              end if;

                           end if;
                        else

                           if single = False then
                              single_message.globe := another_message.globe;
                              single_message.update_time := another_message.update_time;
                              single_message.source_vehicle := another_message.source_vehicle;
                              single_message.v1_position := another_message.v1_position;
                              single := True;
                           else
                              if another_message.update_time > single_message.update_time then
                                 single_message.globe := another_message.globe;
                                 single_message.update_time := another_message.update_time;
                                 single_message.source_vehicle := another_message.source_vehicle;
                                 single_message.v1_position := another_message.v1_position;
                              end if;
                           end if;

                        end if;

                     end if;

                  end loop;

                  if single then -- single mode message sending
                     message.globe := single_message.globe;
                     message.update_time := single_message.update_time;
                     message.source_vehicle := single_message.source_vehicle;
                     message.sending_vehicle := Vehicle_No;
                     message.v1_position := single_message.v1_position;
                     Send (message);
                  elsif same_group then -- dual mode message sending, and only same group vehicles
                     message.globe := same_group_message.globe;
                     message.update_time := same_group_message.update_time;
                     message.source_vehicle := same_group_message.source_vehicle;
                     message.sending_vehicle := Vehicle_No;
                     Send (message);
                  elsif diff_group then -- dual mode message sending, but also different group vehicles
                     message.globe := diff_group_message.globe;
                     message.update_time := diff_group_message.update_time;
                     message.source_vehicle := diff_group_message.source_vehicle;
                     message.sending_vehicle := Vehicle_No;
                     message.v1_position := diff_group_message.v1_position;
                     Send (message);
                  end if;
               end if;

               -----------
               -- Step2 -- Policies for vehicles' movements in different energy stage and how they get recharged
               -----------

               -- energy state, range from 0 to 1, reveal whether this vehicle should do at different energy state.
               current_energy := Current_Charge;

               -- determine whetehr vehicle 2 should be the guider due to they are in the single globe mode or dual globe mode
               if dual_globes_detected then
                  anchor_vehicle_number := 2;
               else
                  anchor_vehicle_number := 1;
               end if;

               -- guider thruttle set
               if Vehicle_No <= anchor_vehicle_number then
                  Set_Destination (message.globe);
                  if abs (Position - message.globe) <= Energy_Globe_Detection then
                     Set_Throttle (0.8);
                  else
                     Set_Throttle (0.9);
                  end if;

               else

                  -- we need to set different energy policy in the dual mode
                  -- they are more hungry in dual mode
                  if dual_globes_detected then
                     if current_energy <= 0.5 then
                        Set_Destination (message.globe);
                        Set_Throttle (1.0);
                     else
                        -- orbit loop
                        if message.point_of_track < 8 then
                           waiting_position := message.globe + orbit_matrix (Vehicle_No mod 3, message.point_of_track mod 8);
                           Set_Destination (waiting_position);
                           if abs (Position - waiting_position) <= Energy_Globe_Detection then
                              message.point_of_track := message.point_of_track + 1;
                           else
                              null;
                           end if;
                        else
                           message.point_of_track := 0;
                           waiting_position := message.globe + orbit_matrix (Vehicle_No mod 3, message.point_of_track mod 8);
                           Set_Destination (waiting_position);
                        end if;

                        if current_energy > 0.8 then
                           Set_Throttle (0.7);
                        elsif current_energy <= 0.8 and then current_energy > 0.5 then
                           Set_Throttle (0.4);
                        end if;
                     end if;
                  else
                     if current_energy <= 0.2 then
                        Set_Destination (message.globe);
                        Set_Throttle (1.0);
                     else

                        if message.point_of_track < 8 then
                           waiting_position := message.globe + orbit_matrix (Vehicle_No mod 3, message.point_of_track mod 8);
                           Set_Destination (waiting_position);
                           if abs (Position - waiting_position) <= Energy_Globe_Detection then
                              message.point_of_track := message.point_of_track + 1;
                           else
                              null;
                           end if;
                        else
                           message.point_of_track := 0;
                           waiting_position := message.globe + orbit_matrix (Vehicle_No mod 3, message.point_of_track mod 8);
                           Set_Destination (waiting_position);
                        end if;

                        if current_energy > 0.8 then
                           Set_Throttle (0.7);
                        elsif current_energy <= 0.8 and then current_energy > 0.5 then
                           Set_Throttle (0.4);
                        elsif current_energy <= 0.5 and then current_energy > 0.2 then
                           Set_Throttle (0.7);
                        end if;
                     end if;
                  end if;

               end if;

            end;
         end loop Outer_task_loop;

      end select;

   exception
      when E : others => Show_Exception (E);

   end Vehicle_Task;

end Vehicle_Task_Type;
