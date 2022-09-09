function [cmd_vel_table, out_vel_table] = standardize_tables(path_to_dir_table, train, time_step_seconds, save_tables)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
if train
    cmd_vel_table_path = path_to_dir_table + "\\cmd_vel_train";
    out_vel_table_path = path_to_dir_table + "\\out_vel_train";
else
    cmd_vel_table_path = path_to_dir_table + "\\cmd_vel_test";
    out_vel_table_path = path_to_dir_table + "\\out_vel_test";
end

cmd_vel_table = readtable(cmd_vel_table_path+".csv");
out_vel_table = readtable(out_vel_table_path+".csv");

cmd_vel_table.x_time = cmd_vel_table.x_time / 1e9;
cmd_vel_table.x_time = cmd_vel_table.x_time - cmd_vel_table.x_time(1);

out_vel_table.x_time = out_vel_table.x_time / 1e9;
out_vel_table.x_time = out_vel_table.x_time - out_vel_table.x_time(1);

end_time_range = min(out_vel_table.x_time(end), cmd_vel_table.x_time(end));
time_range = 0:time_step_seconds:end_time_range;

cmd_vel_table = interp1(cmd_vel_table.x_time, cmd_vel_table.field, time_range);
cmd_vel_table = vertcat(cmd_vel_table, time_range);

out_vel_table = interp1(out_vel_table.x_time, out_vel_table.field, time_range);
out_vel_table = vertcat(out_vel_table, time_range);

if save_tables
    writetable(cmd_vel_table , new_table_name+"_standardized.csv", 'Delimiter', ',');
    writetable(new_table, out_vel_table_path+"_standardized.csv", 'Delimiter', ',');
end

