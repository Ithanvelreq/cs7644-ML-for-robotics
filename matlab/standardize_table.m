function [table] = standardize_table(table, name)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
table.x_time = table.x_time / 1e9;
table.x_time = table.x_time - table.x_time(1);
writetable(table, name, 'Delimiter', ',')
end

