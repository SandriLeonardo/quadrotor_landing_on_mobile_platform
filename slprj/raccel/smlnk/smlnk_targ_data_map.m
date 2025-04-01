    function targMap = targDataMap(),

    ;%***********************
    ;% Create Parameter Map *
    ;%***********************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 1;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc paramMap
        ;%
        paramMap.nSections           = nTotSects;
        paramMap.sectIdxOffset       = sectIdxOffset;
            paramMap.sections(nTotSects) = dumSection; %prealloc
        paramMap.nTotData            = -1;

        ;%
        ;% Auto data (rtP)
        ;%
            section.nData     = 3;
            section.data(3)  = dumData; %prealloc

                    ;% rtP.Sample_Time
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtP._IC
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

                    ;% rtP._IC_mctvldpkh3
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 7;

            nTotData = nTotData + section.nData;
            paramMap.sections(1) = section;
            clear section


            ;%
            ;% Non-auto Data (parameter)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        paramMap.nTotData = nTotData;



    ;%**************************
    ;% Create Block Output Map *
    ;%**************************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 1;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc sigMap
        ;%
        sigMap.nSections           = nTotSects;
        sigMap.sectIdxOffset       = sectIdxOffset;
            sigMap.sections(nTotSects) = dumSection; %prealloc
        sigMap.nTotData            = -1;

        ;%
        ;% Auto data (rtB)
        ;%
            section.nData     = 14;
            section.data(14)  = dumData; %prealloc

                    ;% rtB.krtg4hkewq
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtB.hx4zbpyorm
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 6;

                    ;% rtB.hfcc51pulu
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 12;

                    ;% rtB.nwsu2llks5
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 13;

                    ;% rtB.aitunyvbo3
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 14;

                    ;% rtB.a3uqevckku
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 23;

                    ;% rtB.mdqy0sybu0
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 25;

                    ;% rtB.jxcagdmfiz
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 29;

                    ;% rtB.mg3dqlxqgc
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 32;

                    ;% rtB.bkbzioulom
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 35;

                    ;% rtB.ikfxwn30vz
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 59;

                    ;% rtB.cu04fobhdc
                    section.data(12).logicalSrcIdx = 11;
                    section.data(12).dtTransOffset = 65;

                    ;% rtB.h04naoubyp
                    section.data(13).logicalSrcIdx = 12;
                    section.data(13).dtTransOffset = 66;

                    ;% rtB.dzp0ufmz11
                    section.data(14).logicalSrcIdx = 13;
                    section.data(14).dtTransOffset = 67;

            nTotData = nTotData + section.nData;
            sigMap.sections(1) = section;
            clear section


            ;%
            ;% Non-auto Data (signal)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        sigMap.nTotData = nTotData;



    ;%*******************
    ;% Create DWork Map *
    ;%*******************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 4;
        sectIdxOffset = 1;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc dworkMap
        ;%
        dworkMap.nSections           = nTotSects;
        dworkMap.sectIdxOffset       = sectIdxOffset;
            dworkMap.sections(nTotSects) = dumSection; %prealloc
        dworkMap.nTotData            = -1;

        ;%
        ;% Auto data (rtDW)
        ;%
            section.nData     = 13;
            section.data(13)  = dumData; %prealloc

                    ;% rtDW.pydj2pulvz
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.h4kbtduxza
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.jacddttya0
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 2;

                    ;% rtDW.jsubjbn3x3
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 3;

                    ;% rtDW.o0rlsmhvnx
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 4;

                    ;% rtDW.mj3ycsnbqz
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 5;

                    ;% rtDW.jlqd5uv2t5
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 6;

                    ;% rtDW.bw0m2qixpq
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 7;

                    ;% rtDW.bgxyound4l
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 8;

                    ;% rtDW.jlfkntbhq3
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 9;

                    ;% rtDW.isxzobxtuk
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 10;

                    ;% rtDW.ki1qusscch
                    section.data(12).logicalSrcIdx = 11;
                    section.data(12).dtTransOffset = 11;

                    ;% rtDW.aswygtmbgv
                    section.data(13).logicalSrcIdx = 12;
                    section.data(13).dtTransOffset = 12;

            nTotData = nTotData + section.nData;
            dworkMap.sections(1) = section;
            clear section

            section.nData     = 10;
            section.data(10)  = dumData; %prealloc

                    ;% rtDW.kvnsin1yg3.LoggedData
                    section.data(1).logicalSrcIdx = 13;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.l25hiwzvr2.LoggedData
                    section.data(2).logicalSrcIdx = 14;
                    section.data(2).dtTransOffset = 3;

                    ;% rtDW.eghn2cm5gy.LoggedData
                    section.data(3).logicalSrcIdx = 15;
                    section.data(3).dtTransOffset = 4;

                    ;% rtDW.guy2kna2li.LoggedData
                    section.data(4).logicalSrcIdx = 16;
                    section.data(4).dtTransOffset = 7;

                    ;% rtDW.gsh54m1bmb.LoggedData
                    section.data(5).logicalSrcIdx = 17;
                    section.data(5).dtTransOffset = 10;

                    ;% rtDW.jvhux0m2vh.LoggedData
                    section.data(6).logicalSrcIdx = 18;
                    section.data(6).dtTransOffset = 11;

                    ;% rtDW.crk1vj15iv.LoggedData
                    section.data(7).logicalSrcIdx = 19;
                    section.data(7).dtTransOffset = 13;

                    ;% rtDW.fsz1bpdsia.AQHandles
                    section.data(8).logicalSrcIdx = 20;
                    section.data(8).dtTransOffset = 15;

                    ;% rtDW.piepfykkqk.LoggedData
                    section.data(9).logicalSrcIdx = 21;
                    section.data(9).dtTransOffset = 16;

                    ;% rtDW.c3ebnl0is0.LoggedData
                    section.data(10).logicalSrcIdx = 22;
                    section.data(10).dtTransOffset = 17;

            nTotData = nTotData + section.nData;
            dworkMap.sections(2) = section;
            clear section

            section.nData     = 5;
            section.data(5)  = dumData; %prealloc

                    ;% rtDW.e3y0cazldt
                    section.data(1).logicalSrcIdx = 23;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.ejujdun315
                    section.data(2).logicalSrcIdx = 24;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.f2x2yaoqgy
                    section.data(3).logicalSrcIdx = 25;
                    section.data(3).dtTransOffset = 2;

                    ;% rtDW.d34l4kjipg
                    section.data(4).logicalSrcIdx = 26;
                    section.data(4).dtTransOffset = 3;

                    ;% rtDW.f3nbwloi5j
                    section.data(5).logicalSrcIdx = 27;
                    section.data(5).dtTransOffset = 4;

            nTotData = nTotData + section.nData;
            dworkMap.sections(3) = section;
            clear section

            section.nData     = 18;
            section.data(18)  = dumData; %prealloc

                    ;% rtDW.njpxizyule
                    section.data(1).logicalSrcIdx = 28;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.cjhm3acixb
                    section.data(2).logicalSrcIdx = 29;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.lxythyf1cb
                    section.data(3).logicalSrcIdx = 30;
                    section.data(3).dtTransOffset = 2;

                    ;% rtDW.nh4tvjogpv
                    section.data(4).logicalSrcIdx = 31;
                    section.data(4).dtTransOffset = 3;

                    ;% rtDW.ba1rd1ajhl
                    section.data(5).logicalSrcIdx = 32;
                    section.data(5).dtTransOffset = 4;

                    ;% rtDW.flkg1gk54m
                    section.data(6).logicalSrcIdx = 33;
                    section.data(6).dtTransOffset = 5;

                    ;% rtDW.orxmg0nmrg
                    section.data(7).logicalSrcIdx = 34;
                    section.data(7).dtTransOffset = 6;

                    ;% rtDW.lbm0o1qa52
                    section.data(8).logicalSrcIdx = 35;
                    section.data(8).dtTransOffset = 7;

                    ;% rtDW.av31m02ta4
                    section.data(9).logicalSrcIdx = 36;
                    section.data(9).dtTransOffset = 8;

                    ;% rtDW.briiwtgmsq
                    section.data(10).logicalSrcIdx = 37;
                    section.data(10).dtTransOffset = 9;

                    ;% rtDW.p104kbp3ah
                    section.data(11).logicalSrcIdx = 38;
                    section.data(11).dtTransOffset = 10;

                    ;% rtDW.fdovcjpqrp
                    section.data(12).logicalSrcIdx = 39;
                    section.data(12).dtTransOffset = 11;

                    ;% rtDW.itne4ou4c4
                    section.data(13).logicalSrcIdx = 40;
                    section.data(13).dtTransOffset = 12;

                    ;% rtDW.ffn3ukflan
                    section.data(14).logicalSrcIdx = 41;
                    section.data(14).dtTransOffset = 13;

                    ;% rtDW.a0uid4l0f4
                    section.data(15).logicalSrcIdx = 42;
                    section.data(15).dtTransOffset = 14;

                    ;% rtDW.i5lo0j2nbo
                    section.data(16).logicalSrcIdx = 43;
                    section.data(16).dtTransOffset = 15;

                    ;% rtDW.jwsyzsowec
                    section.data(17).logicalSrcIdx = 44;
                    section.data(17).dtTransOffset = 16;

                    ;% rtDW.lnsdae4spp
                    section.data(18).logicalSrcIdx = 45;
                    section.data(18).dtTransOffset = 17;

            nTotData = nTotData + section.nData;
            dworkMap.sections(4) = section;
            clear section


            ;%
            ;% Non-auto Data (dwork)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        dworkMap.nTotData = nTotData;



    ;%
    ;% Add individual maps to base struct.
    ;%

    targMap.paramMap  = paramMap;
    targMap.signalMap = sigMap;
    targMap.dworkMap  = dworkMap;

    ;%
    ;% Add checksums to base struct.
    ;%


    targMap.checksum0 = 1694352703;
    targMap.checksum1 = 1627899302;
    targMap.checksum2 = 262459659;
    targMap.checksum3 = 2132616286;

