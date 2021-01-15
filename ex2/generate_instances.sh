for i in 4 8 16 32 64 128 256 512 1024 2048 4096 8192; do
  ./generate_random_Kn.tcl $i > instances/K${i}.dimacs;
done

