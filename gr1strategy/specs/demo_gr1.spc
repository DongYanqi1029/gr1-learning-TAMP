# Variables
ENV: ;
SYS: goto_Region1 goto_Region2 goto_Region3;

# Specifications
ENVINIT: ;
ENVTRANS: ;
ENVGOAL: ;

# goto_Region1 & !goto_Region2 & !goto_Region3
SYSINIT: goto_Region1 & !goto_Region2 & !goto_Region3;
SYSTRANS: [] ((goto_Region1 & !goto_Region2 & !goto_Region3) | (!goto_Region1 & goto_Region2 & !goto_Region3) | (!goto_Region1 & !goto_Region2 & goto_Region3))
# & [] (goto_Region1 -> (goto_Region1' | goto_Region2' | goto_Region3'))
# & [] (goto_Region2 -> (goto_Region1' | goto_Region2' | goto_Region3'))
# & [] (goto_Region3 -> (goto_Region1' | goto_Region2' | goto_Region3'))
;
SYSGOAL: []<>goto_Region1
& []<>goto_Region2
& []<>goto_Region3;
