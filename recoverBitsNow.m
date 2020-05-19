function [ ret2,csi ] = recoverBitsNow( ofdmDemodData,Nsym,Nr,Nsts,HCDPiofK,noiseVarEst,mcsTable,pNcbpssi,cfgNHT10)
%UNTITLED11 Summary of this function goes here
%   Detailed explanation goes here
eqDataSym = ones(48,Nsym);
csiData= ones(48,Nsym);
% Equalization
for i = 1: Nsym
    for r = 1:Nr
        for s = 1:Nsts
            
            [eqDataSym(:,i), csiData(:,i)] = wlan.internal.wlanEqualize(ofdmDemodData(:,i), ...
                HCDPiofK(:,i), 'ZF', noiseVarEst);
            
        end
    end
end

csi = csiData;
qamDemodOut = wlan.internal.wlanConstellationDemodulate(eqDataSym, ...
    mcsTable.NBPSCS, noiseVarEst);
% Apply bit-wise CSI
qamDemodOuts=ones(mcsTable.NBPSCS,48,Nsym);
%
for i = 1: Nsym
    for r = 1:Nr
        qamDemodOuts(:,:,i) = bsxfun(@times, ...
            reshape(qamDemodOut(:,i,r), mcsTable.NBPSCS, [], 1), ...
            reshape(csiData(:,i), 1, [])); % [Nbpscs Nsd Nsym]
    end
end
% qamDemodOuts = bsxfun(@times, ...
%               reshape(qamDemodOut, mcsTable.NBPSCS, [], numOFDMSym), ...
%               reshape(csiData, 1, [])); % [Nbpscs Nsd Nsym]


% Deinterleave per OFDM symbol
deintlvrOut = zeros(pNcbpssi*Nsym, 1);
for symIdx = 1:Nsym
    deintlvrOut((symIdx-1)*pNcbpssi+(1:pNcbpssi), 1) = ...
        wlan.internal.wlanBCCDeinterleave(reshape( ...
        qamDemodOuts(:,:,symIdx), [], 1), 'NON_HT', pNcbpssi, mcsTable.NBPSCS);
end

% Channel decoding
nhtDecBits = wlan.internal.wlanBCCDecode(deintlvrOut, mcsTable.Rate);

% Combine bits and Descramble with derived initial State
iniY = nhtDecBits(1:7,1); % Take the first 7 bits for scrambler init
scramInit = [iniY(3)+iniY(7) iniY(2)+iniY(6) iniY(1)+iniY(5) ...
    iniY(4)+iniY(3)+iniY(7) iniY(3)+iniY(2)+iniY(6) ...
    iniY(2)+iniY(1)+iniY(5) iniY(1)+iniY(4)+iniY(3)+iniY(7)];
scramInit = mod(scramInit, 2);
% Remove the padded and tail bits to output only the data bits
descramDataOut = wlan.internal.wlanScramble( ...
    nhtDecBits(1:(16+8*cfgNHT10.PSDULength),:), scramInit);

%   Remove the 16 service bits
rxBits_sim = descramDataOut(17:end);


ret2 = rxBits_sim;
end