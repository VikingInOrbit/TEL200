MODULE Module1

    ! === DIGITAL INPUTS ===
    VAR signaldi di_cube;
    VAR signaldi di_cylinder;
    VAR signaldi di_prism;
    VAR signaldi di_EmergencySituation;
    VAR signaldi di_home;

    ! === COORDINATE REFERENCE ===
    ! Y_LEFT=100, Y_RIGHT=-100, Z_CLEAR=1200, Z_OBJECT=1100
    ! X_CUBE=213, X_CYLINDER=413, X_PRISM=613

    ! === RIGHT ARM (R) HOME POSITION ===
    CONST robtarget HomeR:=[[200,-200,1300],[0,1,0,0],[1,-1,-1,0],[-138.091203512,9E+09,9E+09,9E+09,9E+09,9E+09]];

    ! === CUBE positions (X=213) - RIGHT ARM: moves RIGHT to LEFT ===
    CONST robtarget cube_pickup:=[[213,-100,1100],[0,1,0,0],[0,0,0,0],[-138.091192686,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget cube_deposit:=[[213,100,1100],[0,1,0,0],[0,0,0,0],[-138.091192686,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget cube_approach:=[[213,-100,1200],[0,1,0,0],[0,0,0,0],[-138.091192686,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget cube_depart:=[[213,100,1200],[0,1,0,0],[0,0,0,0],[-138.091192686,9E+09,9E+09,9E+09,9E+09,9E+09]];

    ! === CYLINDER positions (X=413) - RIGHT ARM: moves RIGHT to LEFT ===
    CONST robtarget cylinder_pickup:=[[413,-100,1100],[0,1,0,0],[0,0,0,0],[-138.091192686,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget cylinder_deposit:=[[413,100,1100],[0,1,0,0],[0,0,0,0],[-138.091192686,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget cylinder_approach:=[[413,-100,1200],[0,1,0,0],[0,0,0,0],[-138.091192686,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget cylinder_depart:=[[413,100,1200],[0,1,0,0],[0,0,0,0],[-138.091192686,9E+09,9E+09,9E+09,9E+09,9E+09]];

    ! === PRISM/TRIANGLE positions (X=613) - RIGHT ARM: moves RIGHT to LEFT ===
    CONST robtarget prism_pickup:=[[613,-100,1100],[0,1,0,0],[0,0,0,0],[-138.091192686,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget prism_deposit:=[[613,100,1100],[0,1,0,0],[0,0,0,0],[-138.091192686,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget prism_approach:=[[613,-100,1200],[0,1,0,0],[0,0,0,0],[-138.091192686,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget prism_depart:=[[613,100,1200],[0,1,0,0],[0,0,0,0],[-138.091192686,9E+09,9E+09,9E+09,9E+09,9E+09]];

    PROC main()
        VAR bool cube_moved:=FALSE;
        VAR bool cylinder_moved:=FALSE;
        VAR bool prism_moved:=FALSE;

        ! Bind signal variables to physical I/O
        AliasIO "custom_DI_0", di_cube;
        AliasIO "custom_DI_1", di_cylinder;
        AliasIO "custom_DI_2", di_prism;
        AliasIO "custom_DI_3", di_EmergencySituation;
        AliasIO "custom_DI_4", di_home;

        ! Initialize SmartGripper
        g_Init;
        g_GripOut;
        MoveL HomeR,v500,z50,tool0\WObj:=wobj0;

        ! Main control loop
        WHILE TRUE DO
            ! Check for emergency stop
            IF di_EmergencySituation=0 THEN
                StopMove;
                WaitUntil di_EmergencySituation=1;
                g_Init;
                MoveL HomeR,v500,z50,tool0\WObj:=wobj0;
            ENDIF

            ! Check for home button
            IF di_home=1 THEN
                MoveL HomeR,v500,z50,tool0\WObj:=wobj0;
                WaitTime 0.5;
            ENDIF

            ! Check CUBE switch
            IF di_cube=1 AND cube_moved=FALSE THEN
                MoveCube_RightToLeft;
                cube_moved:=TRUE;
            ELSEIF di_cube=0 THEN
                cube_moved:=FALSE;
            ENDIF

            ! Check CYLINDER switch
            IF di_cylinder=1 AND cylinder_moved=FALSE THEN
                MoveCylinder_RightToLeft;
                cylinder_moved:=TRUE;
            ELSEIF di_cylinder=0 THEN
                cylinder_moved:=FALSE;
            ENDIF

            ! Check PRISM switch
            IF di_prism=1 AND prism_moved=FALSE THEN
                MovePrism_RightToLeft;
                prism_moved:=TRUE;
            ELSEIF di_prism=0 THEN
                prism_moved:=FALSE;
            ENDIF

            WaitTime 0.1;
        ENDWHILE
    ENDPROC

    ! === RIGHT ARM (R): Move CUBE from RIGHT to LEFT (Y=-100 to Y=100) ===
    PROC MoveCube_RightToLeft()
        MoveL HomeR,v500,z50,tool0\WObj:=wobj0;
        MoveL cube_approach,v500,z50,tool0\WObj:=wobj0;
        MoveL cube_pickup,v300,fine,tool0\WObj:=wobj0;
        g_GripIn;
        WaitTime 0.5;
        MoveL cube_approach,v300,z50,tool0\WObj:=wobj0;
        MoveL cube_depart,v500,z50,tool0\WObj:=wobj0;
        MoveL cube_deposit,v300,fine,tool0\WObj:=wobj0;
        g_GripOut;
        WaitTime 0.3;
        MoveL cube_depart,v300,z50,tool0\WObj:=wobj0;
        MoveL HomeR,v500,z50,tool0\WObj:=wobj0;
    ENDPROC

    ! === RIGHT ARM (R): Move CYLINDER from RIGHT to LEFT (Y=-100 to Y=100) ===
    PROC MoveCylinder_RightToLeft()
        MoveL HomeR,v500,z50,tool0\WObj:=wobj0;
        MoveL cylinder_approach,v500,z50,tool0\WObj:=wobj0;
        MoveL cylinder_pickup,v300,fine,tool0\WObj:=wobj0;
        g_GripIn;
        WaitTime 0.5;
        MoveL cylinder_approach,v300,z50,tool0\WObj:=wobj0;
        MoveL cylinder_depart,v500,z50,tool0\WObj:=wobj0;
        MoveL cylinder_deposit,v300,fine,tool0\WObj:=wobj0;
        g_GripOut;
        WaitTime 0.3;
        MoveL cylinder_depart,v300,z50,tool0\WObj:=wobj0;
        MoveL HomeR,v500,z50,tool0\WObj:=wobj0;
    ENDPROC

    ! === RIGHT ARM (R): Move PRISM from RIGHT to LEFT (Y=-100 to Y=100) ===
    PROC MovePrism_RightToLeft()
        MoveL HomeR,v500,z50,tool0\WObj:=wobj0;
        MoveL prism_approach,v500,z50,tool0\WObj:=wobj0;
        MoveL prism_pickup,v300,fine,tool0\WObj:=wobj0;
        g_GripIn;
        WaitTime 0.5;
        MoveL prism_approach,v300,z50,tool0\WObj:=wobj0;
        MoveL prism_depart,v500,z50,tool0\WObj:=wobj0;
        MoveL prism_deposit,v300,fine,tool0\WObj:=wobj0;
        g_GripOut;
        WaitTime 0.3;
        MoveL prism_depart,v300,z50,tool0\WObj:=wobj0;
        MoveL HomeR,v500,z50,tool0\WObj:=wobj0;
    ENDPROC

ENDMODULE