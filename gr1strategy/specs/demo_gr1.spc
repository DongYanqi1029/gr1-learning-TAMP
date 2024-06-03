# Variables
ENV: see_person;
SYS: goto_Region1 goto_Region2 goto_Region3 reach_person;

# Specifications
ENVINIT: ;
ENVTRANS: ;
ENVGOAL: ;

SYSINIT: ;
SYSTRANS: 
[] ((goto_Region1 & !goto_Region2 & !goto_Region3 & !reach_person) | (!goto_Region1 & goto_Region2 & !goto_Region3 & !reach_person) | (!goto_Region1 & !goto_Region2 & goto_Region3 & !reach_person) | (!goto_Region1 & !goto_Region2 & !goto_Region3 & reach_person))
& [] (see_person -> (reach_person))
& [] (!see_person -> !reach_person)
;
SYSGOAL: 
[]<>(goto_Region1 | reach_person)
& []<> (goto_Region2 | reach_person)
& []<> (goto_Region3 | reach_person)
;
