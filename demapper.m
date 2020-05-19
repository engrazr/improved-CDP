function [ output_arg,k2o ] = demapper( DemappedRx,Nsc )
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here

mcs = length(DemappedRx)/48;

k2=1;
k=1;
cnt =0;
cnt2=0;

P5 = 1+1i*0;
P6 = -1+1i*0;
J1=0;
J2=0;

if mcs == 4
    
    outr4 = (ones(length(DemappedRx)/4,1));
    outi4 = (ones(length(DemappedRx)/4,1));
    
    for i = 1 : 192
        if cnt == 2
            cnt=1;
            
        else
            cnt=cnt+1;
            
            
        end
        
        
        
        if cnt == 1
            
            Got1 = abs(-1.6 - DemappedRx(i,:) );
            Got2 = abs( 1.6 - DemappedRx(i,:) );
            Got3 = abs(0.4 - DemappedRx(i,:)) ;
            Got4 = abs(-0.4 - DemappedRx(i,:) );
            [m,TT ]=  min([Got1,Got2,Got3,Got4],[],2);
            NowTT = DemappedRx(i,:);
            TT;
            
            
            
            if TT ==1
                TT=-1.6;
            else
                if TT==2
                    TT=1.6;
                else
                    if TT==3
                        TT = 0.4;
                    else
                        
                        TT = -0.4;
                    end
                end
            end
            
            
        else
            
            Got3 =  abs(0.4 - DemappedRx(i,:) );
            
            Got4 = abs(-0.4 - DemappedRx(i,:) );
            
            
            [m,TT2] =  min([Got3,Got4],[],2) ;
            
            
            if TT2 ==1
                TT2 = 0.4;
            else
                TT2 = -0.4;
                
            end
            
            
            cnt2 = cnt2+1;
            
            
            
            
            if TT == -1.6 && TT2 == 0.4
                if cnt2==1
                    outr4(k2,1) = sqrt(0.9);
                else
                    outi4(k2,1) = sqrt(0.9);
                end
                
            end
            
            if TT == 1.6 && TT2 == 0.4
                
                if cnt2==1
                    outr4(k2,1) = -sqrt(0.9);
                else
                    outi4(k2,1) = -sqrt(0.9);
                end
                
            end
            
            if TT == -0.4 && TT2 == -0.4
                if cnt2==1
                    outr4(k2,1) = sqrt(0.1);
                else
                    outi4(k2,1) = sqrt(0.1);
                end
                
            end
            
            if TT == 0.4 && TT2 == -0.4
                if cnt2==1
                    outr4(k2,1) = -sqrt(0.1);
                else
                    outi4(k2,1) = -sqrt(0.1);
                end
                
            end
            
            
            
        end
        
        if cnt2 == 2
            cnt2=0;
            
            k2 = k2+1;
            
            
        end
        
        
        
        
    end
    T=outr4+1i*outi4;
   
   
    
else
    
    outr = (ones(length(DemappedRx)/2,1));    
    outi = (ones(length(DemappedRx)/2,1));   
    
    for i = 1 : length(DemappedRx)
        
        if mod(i,2)==1
            outr(k,1) = -(1/sqrt(8))* DemappedRx(i,:);
        else
            outi(k,1) = -(1/sqrt(8))* DemappedRx(i,:);
            k=k+1;
        end
        
        
        
    end
    
    
    
    P1=(1/sqrt(8))*2 + 1i*(1/sqrt(8))*2;
    P2=(1/sqrt(8))*2 - 1i*(1/sqrt(8))*2;
    P3=-(1/sqrt(8))*2 + 1i*(1/sqrt(8))*2;
    P4=-(1/sqrt(8))*2 - 1i*(1/sqrt(8))*2;
    
    
    T = ones(Nsc,1);
    
    
    output_args = outr+1i*outi;
    
    G1 = (output_args -P1);
    G2 = (output_args -P2);
    G3 = (output_args -P3);
    G4 = (output_args -P4);
    
    
    G5 = (output_args -P5);
    G6 = (output_args -P6);
    
    
    [val, T(:,1)] =  min([G1(:,1),G2(:,1),G3(:,1),G4(:,1)],[],2) ;
    if Nsc == 48
        
        for i =1:Nsc
            
            if T(i,1)==1
                T(i,1)=P1;
                
            else
                
                if T(i,1)==2
                    T(i,1)=P2;
                    
                else
                    
                    if T(i,1)==3
                        T(i,1)=P3;
                        
                    else
                        
                        T(i,1)=P4;
                        
                    end
                    
                end
                
            end
            
            
        end
        
        
        
        
    end
    
    
    
    
    
    
end


% This section done for the pilot carriors



if Nsc==4
    
    [val, T(:,1)] =  min([G5(:,1),G6(:,1)],[],2) ;
    for i =1:Nsc
        
        if T(i,1)==1
            
            
            T(i,1)=P5;
            
            
        else
            
            
            T(i,1)=P6;
            
            
            
        end
        
        
        
    end
end
k2o=1;
output_arg =T;
end
