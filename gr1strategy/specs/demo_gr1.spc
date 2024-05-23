# Variables
ENV: see_person;
SYS: goto_Region1 goto_Region2 goto_Region3 follow;

# Specifications
ENVINIT: ;
ENVTRANS: ;
ENVGOAL: ;

SYSINIT: ;
SYSTRANS: 
[] ((goto_Region1 & !goto_Region2 & !goto_Region3 & !follow) | (!goto_Region1 & goto_Region2 & !goto_Region3 & !follow) | (!goto_Region1 & !goto_Region2 & goto_Region3 & !follow) | (!goto_Region1 & !goto_Region2 & !goto_Region3 & follow))
& [] (see_person -> (follow))
& [] (!see_person -> !follow)
;
SYSGOAL: 
[]<>(goto_Region1 | follow)
& []<> (goto_Region2 | follow)
& []<> (goto_Region3 | follow)
;
