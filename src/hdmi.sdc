//Copyright (C)2014-2021 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//GOWIN Version: 1.9.8 
//Created Time: 2021-11-11 12:48:43
create_clock -name sys_clk -period 37.037 -waveform {0 18.518} [get_ports {sys_clk}]
create_clock -name cam_pclk -period 65.104 -waveform {0 32.552} [get_ports {cam_pclk}]
create_clock -name vsync_r -period 33333.333 -waveform {0 16666.666} [get_nets {vsync_r}]
create_clock -name clk_pixel -period 39.722 [get_nets {clk_pixel}]
create_clock -name clk_out -period 12.012 [get_nets {clk_out}]

set_clock_groups -asynchronous -group [get_clocks {sys_clk}] -group [get_clocks {cam_pclk}] -group [get_clocks {vsync_r}] -group [get_clocks {clk_pixel}] -group [get_clocks {clk_out}]
