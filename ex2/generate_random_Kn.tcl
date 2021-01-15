#!/usr/bin/tclsh

# Specify required input
set maximum_absolute_weight 1000
if {$argc != 1 && $argc != 2} {
    puts "Usage: ./randKn.tcl <num vertices> <maximum absolute weight>"
    puts "    <num vertices>:              int, at least 3"
    puts "    <maximum absolute weight>:   int, at least 0, default $maximum_absolute_weight"
    puts "Prints a weighted complete graph with specified number of vertices."
    puts "   The edge weights are integers with at most specified absolute value."
    puts "   Among those integers, they are equally distributed"
  return 1;
}

# Parse input
set num_nodes [lindex $argv 0]
if {$argc >= 2} {
    set maxiumm_absolute_weight [lindex $argv 1]
}

# Check input
if {$num_nodes < 3} {
    puts "num vertices must be at least 3!"
    return 1
}
if {$maximum_absolute_weight < 0} {
    puts "maximum absolute weight must be at least 0!"
    return 1
}

set num_edges [ expr {$num_nodes * ($num_nodes-1) / 2} ]
puts "p edge $num_nodes $num_edges"
set smaller_node 1
while {$smaller_node < $num_nodes} {
    set larger_node [expr {$smaller_node+1}]
    while {$larger_node <= $num_nodes} {
	set weight [expr {floor((2*$maximum_absolute_weight+1) * rand()
		                 - $maximum_absolute_weight)}]
	puts "e $smaller_node $larger_node $weight"
	incr larger_node
    }
    incr smaller_node
}
return 0

