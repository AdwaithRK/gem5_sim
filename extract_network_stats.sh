echo > network_stats.txt
grep "packets_injected::total" m5out/stats.txt | sed 's/system.ruby.network.packets_injected::total\s*/packets_injected = /' >> network_stats.txt
grep "packets_received::total" m5out/stats.txt | sed 's/system.ruby.network.packets_received::total\s*/packets_received = /' >> network_stats.txt
grep "average_packet_queueing_latency" m5out/stats.txt | sed 's/system.ruby.network.average_packet_queueing_latency\s*/average_packet_queueing_latency = /' >> network_stats.txt
grep "average_packet_network_latency" m5out/stats.txt | sed 's/system.ruby.network.average_packet_network_latency\s*/average_packet_network_latency = /' >> network_stats.txt
grep "average_packet_latency" m5out/stats.txt | sed 's/system.ruby.network.average_packet_latency\s*/average_packet_latency = /' >> network_stats.txt
grep "flits_injected::total" m5out/stats.txt | sed 's/system.ruby.network.flits_injected::total\s*/flits_injected = /' >> network_stats.txt
grep "flits_received::total" m5out/stats.txt | sed 's/system.ruby.network.flits_received::total\s*/flits_received = /' >> network_stats.txt
grep "average_flit_queueing_latency" m5out/stats.txt | sed 's/system.ruby.network.average_flit_queueing_latency\s*/average_flit_queueing_latency = /' >> network_stats.txt
grep "average_flit_network_latency" m5out/stats.txt | sed 's/system.ruby.network.average_flit_network_latency\s*/average_flit_network_latency = /' >> network_stats.txt
grep "average_flit_latency" m5out/stats.txt | sed 's/system.ruby.network.average_flit_latency\s*/average_flit_latency = /' >> network_stats.txt
grep "average_hops" m5out/stats.txt | sed 's/system.ruby.network.average_hops\s*/average_hops = /' >> network_stats.txt

grep "system.cpu0.numCycles " m5out/stats.txt | sed 's/system.cpu0.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu1.numCycles " m5out/stats.txt | sed 's/system.cpu1.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu2.numCycles " m5out/stats.txt | sed 's/system.cpu2.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu3.numCycles " m5out/stats.txt | sed 's/system.cpu3.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu4.numCycles " m5out/stats.txt | sed 's/system.cpu4.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu5.numCycles " m5out/stats.txt | sed 's/system.cpu5.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu6.numCycles " m5out/stats.txt | sed 's/system.cpu6.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu7.numCycles " m5out/stats.txt | sed 's/system.cpu7.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu8.numCycles " m5out/stats.txt | sed 's/system.cpu8.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu9.numCycles " m5out/stats.txt | sed 's/system.cpu9.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu10.numCycles " m5out/stats.txt | sed 's/system.cpu10.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu11.numCycles " m5out/stats.txt | sed 's/system.cpu11.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu12.numCycles " m5out/stats.txt | sed 's/system.cpu12.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu13.numCycles " m5out/stats.txt | sed 's/system.cpu13.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu14.numCycles " m5out/stats.txt | sed 's/system.cpu14.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu15.numCycles " m5out/stats.txt | sed 's/system.cpu15.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu16.numCycles " m5out/stats.txt | sed 's/system.cpu16.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu17.numCycles " m5out/stats.txt | sed 's/system.cpu17.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu18.numCycles " m5out/stats.txt | sed 's/system.cpu18.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu19.numCycles " m5out/stats.txt | sed 's/system.cpu19.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu20.numCycles " m5out/stats.txt | sed 's/system.cpu20.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu21.numCycles " m5out/stats.txt | sed 's/system.cpu21.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu22.numCycles " m5out/stats.txt | sed 's/system.cpu22.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu23.numCycles " m5out/stats.txt | sed 's/system.cpu23.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu24.numCycles " m5out/stats.txt | sed 's/system.cpu24.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu25.numCycles " m5out/stats.txt | sed 's/system.cpu25.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu26.numCycles " m5out/stats.txt | sed 's/system.cpu26.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu27.numCycles " m5out/stats.txt | sed 's/system.cpu27.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu28.numCycles " m5out/stats.txt | sed 's/system.cpu28.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu29.numCycles " m5out/stats.txt | sed 's/system.cpu29.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu30.numCycles " m5out/stats.txt | sed 's/system.cpu30.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu31.numCycles " m5out/stats.txt | sed 's/system.cpu31.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu32.numCycles " m5out/stats.txt | sed 's/system.cpu32.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu33.numCycles " m5out/stats.txt | sed 's/system.cpu33.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu34.numCycles " m5out/stats.txt | sed 's/system.cpu34.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu35.numCycles " m5out/stats.txt | sed 's/system.cpu35.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu36.numCycles " m5out/stats.txt | sed 's/system.cpu36.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu37.numCycles " m5out/stats.txt | sed 's/system.cpu37.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 

grep "system.cpu38.numCycles " m5out/stats.txt | sed 's/system.cpu38.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu39.numCycles " m5out/stats.txt | sed 's/system.cpu39.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu40.numCycles " m5out/stats.txt | sed 's/system.cpu40.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu41.numCycles " m5out/stats.txt | sed 's/system.cpu41.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu42.numCycles " m5out/stats.txt | sed 's/system.cpu42.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu43.numCycles " m5out/stats.txt | sed 's/system.cpu43.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu44.numCycles " m5out/stats.txt | sed 's/system.cpu44.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu45.numCycles " m5out/stats.txt | sed 's/system.cpu45.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu46.numCycles " m5out/stats.txt | sed 's/system.cpu46.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu47.numCycles " m5out/stats.txt | sed 's/system.cpu47.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu48.numCycles " m5out/stats.txt | sed 's/system.cpu48.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu49.numCycles " m5out/stats.txt | sed 's/system.cpu49.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu50.numCycles " m5out/stats.txt | sed 's/system.cpu50.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu51.numCycles " m5out/stats.txt | sed 's/system.cpu51.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu52.numCycles " m5out/stats.txt | sed 's/system.cpu52.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu53.numCycles " m5out/stats.txt | sed 's/system.cpu53.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu54.numCycles " m5out/stats.txt | sed 's/system.cpu54.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu55.numCycles " m5out/stats.txt | sed 's/system.cpu55.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu56.numCycles " m5out/stats.txt | sed 's/system.cpu56.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu57.numCycles " m5out/stats.txt | sed 's/system.cpu57.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu58.numCycles " m5out/stats.txt | sed 's/system.cpu58.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu59.numCycles " m5out/stats.txt | sed 's/system.cpu59.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu60.numCycles " m5out/stats.txt | sed 's/system.cpu60.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu61.numCycles " m5out/stats.txt | sed 's/system.cpu61.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu62.numCycles " m5out/stats.txt | sed 's/system.cpu62.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 
grep "system.cpu63.numCycles " m5out/stats.txt | sed 's/system.cpu63.numCycles \s*//'| sed 's/# Number of cpu cycles simulated (Cycle)//' >> network_stats.txt 


grep "system.cpu0.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu0.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 

grep "system.cpu1.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu1.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu2.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu2.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt
grep "system.cpu3.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu3.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu4.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu4.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu5.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu5.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu6.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu6.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu7.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu7.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu8.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu8.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu9.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu9.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 

grep "system.cpu10.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu10.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 

grep "system.cpu11.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu11.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu12.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu12.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu13.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu13.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 

grep "system.cpu14.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu14.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu15.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu15.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu16.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu16.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu17.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu17.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu18.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu18.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 

grep "system.cpu19.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu19.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 

grep "system.cpu20.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu20.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu21.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu21.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu22.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu22.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu23.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu23.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu24.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu24.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu25.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu25.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu26.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu26.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu27.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu27.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu28.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu28.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 

grep "system.cpu29.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu29.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 

grep "system.cpu30.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu30.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu31.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu31.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu32.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu32.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu33.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu33.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu34.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu34.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu35.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu35.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu36.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu36.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu37.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu37.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu38.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu38.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 

grep "system.cpu39.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu39.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 

grep "system.cpu40.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu40.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu41.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu41.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu42.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu42.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu43.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu43.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu44.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu44.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu45.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu45.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu46.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu46.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu47.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu47.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu48.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu48.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 

grep "system.cpu49.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu49.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 

grep "system.cpu50.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu50.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu51.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu51.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu52.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu52.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu53.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu53.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu54.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu54.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu55.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu55.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu56.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu56.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu57.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu57.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu58.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu58.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 



grep "system.cpu59.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu59.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu60.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu60.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu61.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu61.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 
grep "system.cpu62.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu62.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 

grep "system.cpu63.exec_context.thread_0.numInsts  " m5out/stats.txt | sed 's/system.cpu63.exec_context.thread_0.numInsts  \s*//'| sed 's/# Number of instructions committed (Count)//' >> network_stats.txt 



grep "system.ruby.l1_cntrl0.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl0.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l1_cntrl1.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl1.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl2.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl2.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl3.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl3.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl4.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl4.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl5.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl5.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l1_cntrl6.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl6.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl7.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl7.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l1_cntrl8.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl8.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl9.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl9.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl10.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl10.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl11.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl11.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl12.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl12.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l1_cntrl13.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl13.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl14.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl14.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l1_cntrl15.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl15.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl16.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl16.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl17.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl17.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl18.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl18.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl19.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl19.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l1_cntrl20.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl20.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl21.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl21.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l1_cntrl22.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl22.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl23.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl23.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl24.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl24.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl25.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl25.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl26.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl26.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l1_cntrl27.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl27.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl28.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl28.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l1_cntrl29.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl29.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl30.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl30.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl31.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl31.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl32.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl32.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl33.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl33.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l1_cntrl34.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl34.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl35.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl35.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l1_cntrl36.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl36.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl37.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl37.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl38.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl38.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl39.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl39.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl40.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl40.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l1_cntrl41.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl41.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl42.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl42.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l1_cntrl43.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl43.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl44.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl44.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl45.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl45.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl46.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl46.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl47.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl47.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l1_cntrl48.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl48.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl49.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl49.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l1_cntrl50.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl50.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl51.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl51.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl52.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl52.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl53.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl53.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl54.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl54.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l1_cntrl55.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl55.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l1_cntrl56.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl56.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l1_cntrl57.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl57.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl58.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl58.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl59.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl59.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl60.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl60.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl61.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl61.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l1_cntrl62.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl62.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l1_cntrl63.L1Dcache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl63.L1Dcache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 



grep "system.ruby.l1_cntrl0.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl0.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l1_cntrl1.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl1.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl2.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl2.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl3.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl3.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl4.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl4.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl5.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl5.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l1_cntrl6.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl6.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl7.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl7.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l1_cntrl8.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl8.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl9.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl9.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl10.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl10.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl11.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl11.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl12.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl12.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l1_cntrl13.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl13.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl14.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl14.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l1_cntrl15.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl15.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl16.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl16.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl17.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl17.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl18.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl18.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl19.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl19.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l1_cntrl20.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl20.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl21.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl21.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l1_cntrl22.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl22.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl23.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl23.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl24.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl24.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl25.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl25.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl26.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl26.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l1_cntrl27.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl27.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl28.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl28.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l1_cntrl29.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl29.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl30.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl30.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl31.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl31.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl32.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl32.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl33.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl33.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l1_cntrl34.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl34.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl35.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl35.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l1_cntrl36.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl36.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl37.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl37.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl38.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl38.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl39.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl39.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl40.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl40.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l1_cntrl41.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl41.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl42.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl42.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l1_cntrl43.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl43.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl44.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl44.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl45.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl45.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl46.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl46.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl47.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl47.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l1_cntrl48.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl48.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl49.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl49.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l1_cntrl50.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl50.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl51.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl51.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl52.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl52.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl53.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl53.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl54.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl54.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l1_cntrl55.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl55.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l1_cntrl56.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl56.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l1_cntrl57.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl57.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl58.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl58.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl59.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl59.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl60.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl60.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l1_cntrl61.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl61.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l1_cntrl62.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl62.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l1_cntrl63.L1Icache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l1_cntrl63.L1Icache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 



grep "system.ruby.l2_cntrl0.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl0.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l2_cntrl1.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl1.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl2.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl2.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl3.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl3.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl4.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl4.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl5.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl5.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l2_cntrl6.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl6.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl7.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl7.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l2_cntrl8.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl8.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl9.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl9.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl10.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl10.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl11.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl11.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl12.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl12.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l2_cntrl13.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl13.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl14.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl14.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l2_cntrl15.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl15.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl16.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl16.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl17.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl17.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl18.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl18.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl19.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl19.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l2_cntrl20.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl20.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl21.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl21.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l2_cntrl22.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl22.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl23.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl23.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl24.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl24.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl25.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl25.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl26.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl26.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l2_cntrl27.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl27.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl28.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl28.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l2_cntrl29.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl29.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl30.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl30.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl31.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl31.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl32.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl32.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl33.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl33.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l2_cntrl34.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl34.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl35.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl35.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l2_cntrl36.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl36.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl37.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl37.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl38.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl38.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl39.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl39.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl40.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl40.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l2_cntrl41.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl41.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl42.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl42.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l2_cntrl43.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl43.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl44.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl44.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl45.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl45.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl46.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl46.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl47.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl47.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l2_cntrl48.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl48.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl49.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl49.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l2_cntrl50.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl50.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl51.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl51.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl52.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl52.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl53.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl53.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl54.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl54.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l2_cntrl55.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl55.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l2_cntrl56.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl56.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 


grep "system.ruby.l2_cntrl57.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl57.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl58.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl58.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl59.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl59.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl60.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl60.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 

grep "system.ruby.l2_cntrl61.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl61.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l2_cntrl62.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl62.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 
grep "system.ruby.l2_cntrl63.L2cache.m_demand_misses  " m5out/stats.txt | sed 's/system.ruby.l2_cntrl63.L2cache.m_demand_misses  \s*//'| sed 's/ # Number of cache demand misses (Unspecified)//' >> network_stats.txt 




