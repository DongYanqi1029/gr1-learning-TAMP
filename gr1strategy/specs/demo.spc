# Variables
ENV: ;
SYS: r1 r2 r3;

# Specifications
ENVINIT: ;
ENVTRANS: ;
ENVGOAL: ;

# r1 & !r2 & !r3
SYSINIT: r1 & !r2 & !r3;
SYSTRANS: [] ((r1 & !r2 & !r3) | (!r1 & r2 & !r3) | (!r1 & !r2 & r3))
& [] (r1 -> (r1' | r2' | r3'))
& [] (r2 -> (r1' | r2' | r3'))
& [] (r3 -> (r1' | r2' | r3'))
;
SYSGOAL: []<>r1
& []<>r2
& []<>r3;