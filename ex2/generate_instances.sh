for i in 5 6 7 9 10 11 12 13 14 15 ; do
  ./generate_random_Kn.tcl $i > instances/K${i}.dimacs;
done

