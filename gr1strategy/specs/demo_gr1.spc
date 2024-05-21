# Variables
ENV: see_person;
SYS: goto_Region1 goto_Region2 goto_Region3 follow;

# Specifications
ENVINIT: ;
ENVTRANS: ;
ENVGOAL: ;

# (!goto_Region1' & !goto_Region2' & !goto_Region3') & 
SYSINIT: ;
SYSTRANS: [] ((goto_Region1 & !goto_Region2 & !goto_Region3) | (!goto_Region1 & goto_Region2 & !goto_Region3) | (!goto_Region1 & !goto_Region2 & goto_Region3) | follow)
& [] (see_person -> ((!goto_Region1 & !goto_Region2 & !goto_Region3) & follow))
;
SYSGOAL: []<>(goto_Region1 | follow)
& []<> (goto_Region2 | follow)
& []<> (goto_Region3 | follow);
