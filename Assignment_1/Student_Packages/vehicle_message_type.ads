-- Suggestions for packages which might be useful:

with Ada.Real_Time;         use Ada.Real_Time;
--  with Vectors_3D;            use Vectors_3D;
with Swarm_Structures_Base; use Swarm_Structures_Base;
package Vehicle_Message_Type is

   -- Replace this record definition by what your vehicles need to communicate.

   type Inter_Vehicle_Messages is
      record
         -- the position of an energy globe that the vehicle can get recharged

         -- for stage B
         globe : Positions;
         -- the update time for this message
         update_time : Time;
         -- the stage of the vehicle in its orbit, which includes 7 stages
         point_of_track : Integer;

         -- for stage C
         -- the source of this message
         source_vehicle : Integer;
         -- the last vehicle that sent this message to thi current vehicle
         sending_vehicle : Integer;
         -- the position of vehicle_1 so that it can reveal the postion of a energy globe in this way.
         v1_position : Positions;
      end record;

end Vehicle_Message_Type;
