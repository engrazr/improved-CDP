% THIS CODE IS FOR CHANNEL ESTIMATION IN IEEE 802.11p PHYSICAL LAYER
% USE Matlabb version 2016b
% Please find the detail about code in below article and cite it if you use this code in your work.
% Wang, T.; Hussain, A.; Cao, Y.; Gulomjon, S. An Improved Channel Estimation Technique for IEEE 802.11p Standard in Vehicular Communications. Sensors 2019, 19, 98.

clc;
EsNodB = 1:35;
snr = 10.^(EsNodB/10);
Smp=64;
mcs =2;
psduLen =50*20;% byte pay load size = psduLen*8 bits

W=0;

Ds = 1200; % Hz
0; % doppler spread
% Maximum Doppler shift, Hz
c = 3e8*3.6;                        % Speed of light, Km/hr
fc = 5.9e9;                         % Carrier frequency, Hz


idleTime = 0;
numPkts = 10;
winTransTime = 0; % No windowing
chDelay = 00; % arbitrary delay to account for all channel profiles

h =figure;
grid on ;
hold on;
xlabel('SNR (dB)');
ylabel('PER');

ax=gca;
% ylim([1e-3 1]);
ax.YScale ='log';


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

MC = 100; % number of frames
MSErrorLS=zeros(1,length(EsNodB));
MSErroriCDP=zeros(1,length(EsNodB));
MSErrorCDP=zeros(1,length(EsNodB));
MSErrorDFT=zeros(1,length(EsNodB));
MSErrorSTA=zeros(1,length(EsNodB));
BERLS = zeros(1,length(EsNodB));
BERiCDP = zeros(1,length(EsNodB));
BERCDP = zeros(1,length(EsNodB));
BERDFT = zeros(1,length(EsNodB));
BERSTA = zeros(1,length(EsNodB));
NumOfErrorBitLS    = zeros(1,MC);
NumOfErrorBitiCDP  = zeros(1,MC);
NumOfErrorBitCDP    = zeros(1,MC);
NumOfErrorBitLDFT    = zeros(1,MC);
NumOfErrorBitSTA    = zeros(1,MC);

NumOfErrorBitazr =zeros(1,MC);
jj = zeros(1,length(EsNodB));

chanMdl = 'E';

cfgNHT10 = wlanNonHTConfig('PSDULength', psduLen);
cfgNHT10.Modulation = 'OFDM';
cfgNHT10.ChannelBandwidth = 'CBW10';    % 10 MHz channel bandwidth
cfgNHT10.MCS = mcs;
fs10 = helperSampleRate(cfgNHT10);     % Baseband sampling rate for 10 MHz
trms = 0.4e-6;
fs = fs10;

fd =Ds;

fadingMargin = 12; % dB
%chan10 = stdchan(1/fs10, fd, chantype);
disp(['Speed of unit = ' num2str(c*fd/5.9e9) ' Km/hr at ' num2str(fc/1e9) ' GHz']);

chan10 = stdchan(1/fs10, fd, ['hiperlan2' chanMdl]);
chan10.StoreHistory = 1;
s = rng(98765);
ofdmInfo = wlan.internal.wlanGetOFDMConfig(cfgNHT10.ChannelBandwidth, 'Long', 'Legacy');
Nst = numel(ofdmInfo.DataIndices)+numel(ofdmInfo.PilotIndices); % Number of occupi}[ed subcarriers

beta = 17/9;
i=0;
nFFT = 64;
nCP = 16;
mcsTable = wlan.internal.getRateTable(cfgNHT10);
pNcbpssi = mcsTable.NCBPS;

cfgRec = wlanRecoveryConfig('EqualizationMethod', 'ZF');

for ii = 1:length(EsNodB)% iterate for a particular snr
      
    ChMSE_LSo = 0;
    ChMSE_iCDP=0;
    ChMSE_CDP=0;
    ChMSE_DFT=0;
    ChMSE_STA=0;
    numPacketErrors = 0;
    numPkt = 0; % Index of packet transmitted
    packetErrorCDP =0;
    packetErrorSTA = 0;
    packetErrorAZR= 0;
    packetErrorLS = 0;
    packetErrorDFT = 0;
    NumpacketErrorCDP =0;
    NumpacketErrorSTA = 0;
    NumpacketErrorAZR =0;
    NumpacketErrorLS =0;
    NumpacketErrorDFT = 0;
    enableFE = true;
    inpPSDU = randi([0 1], 8*cfgNHT10.PSDULength, 1);
    ind = wlanFieldIndices(cfgNHT10);
    tx = wlanWaveformGenerator(inpPSDU,cfgNHT10, 'IdleTime', idleTime,...
        'NumPackets', numPkts, 'WindowTransitionTime', winTransTime);
    awgnChannel = comm.AWGNChannel;
    awgnChannel.NoiseMethod = 'Signal to noise ratio (SNR)';
    awgnChannel.SignalPower = 1;              % Unit power
    awgnChannel.SNR = EsNodB (ii)-10*log10(ofdmInfo.FFTLength/Nst); % Account for energy in nulls
    
    for mc = 1:MC
        
        chDelay = 100;
        padTx = [tx; zeros(chDelay, 1)];
        numRxAnts=1;
        XDD=reshape(tx,80,[]);% 80x6=480
        [ofdmSize,ofdmSymbols] = size(XDD);
        s = validateConfig(cfgNHT10);% tells the number of data ofdm symbols
        numOFDMSym = s.NumDataSymbols;
        z=1;% accouts for the L-SIG pilot Symbol
        refPilots = wlan.internal.nonHTPilots(s.NumDataSymbols,z);
        % Get OFDM configuration
        [cfgOFDM,dataInd,pilotInd] = wlan.internal.wlanGetOFDMConfig( ...
            cfgNHT10.ChannelBandwidth, 'Long', 'Legacy');
        % Cross validate inputs
        numST = numel([dataInd; pilotInd]); % Total number of occupied subcarriers
        % Cross-validation between inputs
        minInputLen = numOFDMSym*(cfgOFDM.FFTLength+cfgOFDM.CyclicPrefixLength);
        rx = filter(chan10, padTx);
        rx = awgnChannel(rx);
        chDelay = chan10.ChannelFilterDelay;
        rx = rx(chDelay+1:end,:);
        
        lltftx = tx(ind.LLTF(1):ind.LLTF(2),:);
        lstftx = tx(ind.LSTF(1):ind.LSTF(2),:);
        
        lltfDemodtx = wlanLLTFDemodulate(lltftx, cfgNHT10, 1);
        lstfDemodtx = wlanLLTFDemodulate(lstftx, cfgNHT10, 1);
        
        lltf = rx(ind.LLTF(1):ind.LLTF(2),:);
        lstf = rx(ind.LSTF(1):ind.LSTF(2),:);
        
        lltfDemod = wlanLLTFDemodulate(lltf, cfgNHT10, 1);
        lstfDemod = wlanLLTFDemodulate(lstf, cfgNHT10, 1);
        
        
        %%%%%%%%%%%%%%%%%%% LEAST SQUARE ESTIMATION %%%%%%%%%%%%%%%%%
        lltfo = lltfReference(1); % Get reference subcarriers
        ls = lltfDemod./lltfDemodtx;%repmat(lltfo,1,size(lltfDemod,2),numRxAnts); % Least-square estimate
        
        
        HhatLSo = mean(ls,2); % Average over the symbols
        HhatLSof = [zeros(6,1);HhatLSo;zeros(5,1)];
        HhatLSx = ifft(HhatLSof,64);
        HhatLS = fft(HhatLSx(1:64),64);
        
        symOffset = 0.75;
        eqMethod  = 'ZF';
        pilotPhaseTracking = 'PreEQ';
        
        coder.internal.errorIf(size(rx(ind.NonHTData(1):ind.NonHTData(2),:), 1) < minInputLen, ...
            'wlan:wlanNonHTDataRecover:ShortNHTDataInput', minInputLen);
        chanEst = HhatLSo; %HhatLS(7:58,1);
        
        noiseVarEst = helperNoiseEstimate(lltfDemod);
        
        
        rxBits_LS = wlanNonHTDataRecover(rx(ind.NonHTData(1):ind.NonHTData(2),:), chanEst, noiseVarEst, cfgNHT10, cfgRec);
        
        rxNonHTData = rx(ind.NonHTData(1):ind.NonHTData(2),:);
        txNonHTData = tx(ind.NonHTData(1):ind.NonHTData(2),:);
        rxi = rxNonHTData(1:minInputLen, :);
        txi = txNonHTData(1:minInputLen, :);
        FFTLen    = cfgOFDM.FFTLength;
        CPLen     = cfgOFDM.CyclicPrefixLength;
        numRx     = size(rxi, 2);
        numTx     = size(txi, 2);
        symOffset = round(symOffset * CPLen);
        
        % Remove cyclic prefix
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if isscalar(CPLen)
            numSym = size(rxi, 1)/(FFTLen + CPLen);
            
            %%%%%%%%%%%%%%%%%% RX SIDE %%%%%%%%%%%%%%
            
            inputIn3Drx = reshape(rxi, [(FFTLen + CPLen) numSym numRx]);
            postCPRemovalrx = inputIn3Drx([CPLen+1:FFTLen+symOffset, symOffset+1:CPLen], :, :);
            
            %%%%%%%%%%%%%%%%%% TX SIDE %%%%%%%%%%%%%%
            
            inputIn3Dtx = reshape(txi, [(FFTLen + CPLen) numSym numTx]);
            postCPRemovaltx = inputIn3Dtx([CPLen+1:FFTLen+symOffset, symOffset+1:CPLen], :, :);
            
            
            
        else
            numSym = length(CPLen);
            
            postCPRemovalrx = coder.nullcopy(complex(zeros(FFTLen, numSym, numRx)));
            postCPRemovaltx = coder.nullcopy(complex(zeros(FFTLen, numSym, numTx)));
            
            
            currentIdx = 0;
            for symIdx = 1:numSym
                
                postCPRemovalrx(:, symIdx, :) = rxi(currentIdx + ...
                    [CPLen(symIdx)+1:FFTLen+symOffset(symIdx), symOffset(symIdx)+1:CPLen(symIdx)], :);
                
                postCPRemovaltx(:, symIdx, :) = txi(currentIdx + ...
                    [CPLen(symIdx)+1:FFTLen+symOffset(symIdx), symOffset(symIdx)+1:CPLen(symIdx)], :);
                
                
                
                
                currentIdx = currentIdx + CPLen(symIdx) + FFTLen;
            end
        end
        
        % Denormalization RX
        postCPRemovalrx = postCPRemovalrx / cfgOFDM.NormalizationFactor;
        postCPRemovaltx = postCPRemovaltx / cfgOFDM.NormalizationFactor;
        % FFT
        postFFTrx = fft(postCPRemovalrx, [], 1);
        postFFTtx = fft(postCPRemovaltx, [], 1);
        % FFT shift
        if isreal(postFFTrx)
            postShiftrx = complex(fftshift(postFFTrx, 1), 0);
        else
            postShiftrx = fftshift(postFFTrx,1);
        end
        % FFT shift
        if isreal(postFFTtx)
            postShifttx = complex(fftshift(postFFTtx, 1), 0);
        else
            postShifttx = fftshift(postFFTtx,1);
        end
        % Phase rotation on frequency subcarriers
        postShiftrx = bsxfun(@rdivide, postShiftrx, cfgOFDM.CarrierRotations);
        postShifttx = bsxfun(@rdivide, postShifttx, cfgOFDM.CarrierRotations);
        % Output data
        ofdmDemodData = postShiftrx(cfgOFDM.DataIndices, :, :);
        ofdmDemodDatatx = postShifttx(cfgOFDM.DataIndices, :, :);
        % Output pilots
        ofdmDemodPilots = postShiftrx(cfgOFDM.PilotIndices, :, :);
        ofdmDemodPilotstx = postShifttx(cfgOFDM.PilotIndices, :, :);
        
        %%
        chanEstData = chanEst(dataInd,:,:);
        chanEstPilots = chanEst(pilotInd,:,:);
        % Estimate CPE and phase correct symbols
        % cpe = wlan.internal.commonPhaseErrorEstimate(ofdmDemodPilots, ...
        %         chanEstPilots, refPilots);
        rxPilots = ofdmDemodPilots;
        Nsts = size(chanEstPilots,2);
        %%%%%%%%%%%%%%%%%%%% DFT BASED CHANNEL ESTIMATION %%%%%%%%%%%%%%%%%%%%%%%%%
        HhatDFT = ifft(HhatLS,64);
        HhatDFT = [HhatDFT(1:18);zeros(40,1)];
        HhatDFT = fft(HhatDFT,64);
        rxBits_LDFT = wlanNonHTDataRecover(rx(ind.NonHTData(1):ind.NonHTData(2),:), HhatDFT(7:58,1), noiseVarEst, cfgNHT10, cfgRec);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%HERE DO CDP ESTIMATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        ShatRiofK = ofdmDemodData;
        [Nsc,Nsym,Nr] = size(ShatRiofK);
        Np=4;
        csiDatai = complex(zeros(Nsc,Nsym,Nr));
        csiDataticki = complex(zeros(Nsc,Nsym,Nr));
        csiDatatickticki = complex(zeros(Nsc,Nsym,Nr));
        
        XhatSym = complex(zeros(Nsc,Nsym,Nr));
        XhattickSym = complex(ones(Nsc,Nsym,Nr));
        XhatticktickSym = complex(ones(Nsc,Nsym,Nr));
        
        HiofK = (ones(Nsc,Nsym));
        HiofKoriginal = (ones(Nsc,Nsym));
        
        HiofKsta = (ones(52,Nsym));
        Hupdate = (ones(52,Nsym));
        HupdateAzr = (ones(52,Nsym));
        
        DemappedTx = ones(pNcbpssi,Nsym);
        DemappedRx = ones(pNcbpssi,Nsym);
        DemappedSC = ones(pNcbpssi,Nsym);
        DemappedSSC = ones(pNcbpssi,Nsym);
        DemappedRxstadata = ones(pNcbpssi,Nsym);
        DemappedRxstapilot = ones(8,Nsym);
        
        
        HCDPiofK = (ones(Nsc,Nsym));
        h_estiSEMI0 = HCDPiofK;
        Hsta = (ones(Nsc,Nsym));
        Hazr = (ones(Nsc,Nsym));
        X=(ones(Nsc,Nsym));
        Xtest=(ones(Nsc,Nsym));
        
        
        Xstadata =(ones(Nsc,Nsym));
        Xstapilot =(ones(4,Nsym));
        XSC=(ones(Nsc,Nsym));
        XSSC=(ones(Nsc,Nsym));
        XSSC0 = lltfDemodtx(:,2);
        XSSC0 = XSSC0(dataInd,1,:);
        
        XT=(ones(Nsc,Nsym));
        XZ=(ones(Nsc,Nsym));
        
        
        ShattickCiminus1 = complex(ones(Nsc,Nsym,Nr));
        ShatticktickCiminus1 = complex(ones(Nsc,Nsym,Nr));
        ShatTiofKK = ofdmDemodDatatx;% complex(zeros(Np,Nsym,Nr));
        ShatTiofK = ShatTiofKK;% complex(zeros(Np,Nsym,Nr));
        SCiofK = complex(ones(Nsc,Nsym,Nr));
        ShatRiofK = ofdmDemodData;
        SSCiofK = complex(ones(Nsc,Nsym,Nr));
        SCiofK0=complex(ones(Nsc,Nsym,Nr));
        
        chanEstCDPData = complex(ones(Nsc,Nsym,Nr));
        ShatTiofKstadata =complex(ones(Nsc,Nsym,Nr));
        ShatTiofKstapilot = complex(ones(4,Nsym,Nr));
        ShatRiofKsta = [postShiftrx(7:32,:);postShiftrx(34:59,:)];
        ShatTiofKstao = [postShifttx(7:32,:);postShifttx(34:59,:)];
        ShatTiofKsta =ShatRiofKsta;
        ShatTiofKstaFix =complex(ones(64,Nsym,Nr));
        Hooo = ShatRiofKsta;
        chanEstCDPData = chanEst(dataInd,:,:);
        HCDPiofK0(:,1,1) = chanEst(dataInd,:,:);
        HCDPiofKsta = ShatRiofKsta;
        HCDPiofKazr = ShatRiofKsta;
        ShatRiofKstatemp = ShatRiofKsta;
        Hh = ShatRiofKstatemp;
        HCDPiofKsta0 = chanEst;
        SU = real(lltfDemod(:,2));
        SU = SU(dataInd,:,:);
        Beta =2;
        Wlembda = 1/(2*Beta+1);
        Nsc=48;
        for r = 1:Nr
            for s =1:Nsts
                for i = 1:Nsym
                    
                    if i==1
                        ShatTiofKsta(:,i,r) = ShatRiofKsta(:,i,r)./HCDPiofKsta0;
                        ShatTiofKstadata(:,i,r) = ShatTiofKsta(dataInd,i,r);
                        ShatTiofKstapilot(:,i,r) = ShatTiofKsta(pilotInd,i,r);
                        AAA = wlan.internal.wlanConstellationDemodulate(ShatTiofKstadata(:,i,r),mcsTable.NBPSCS,1);
                        DemappedRxstadata(:,i) = wlan.internal.wlanConstellationDemodulate(ShatTiofKstadata(:,i,r),mcsTable.NBPSCS,1);
                        [Xstadata(:,i),k2] = demapper( DemappedRxstadata(:,i) ,Nsc);
                        DemappedRxstapilot(:,i) = wlan.internal.wlanConstellationDemodulate(ShatTiofKstapilot(:,i,r),2,1);Xstapilot(:,i) = demapper( DemappedRxstapilot(:,i),Np );
                        ShatRiofKstatemp(dataInd,i,r) = Xstadata(:,i);
                        ShatRiofKstatemp(pilotInd,i,r) = Xstapilot(:,i);
                        HiofKsta(:,i) = ShatRiofKsta(:,i,r)./ ShatRiofKstatemp(:,i);
                        Hupdate(:,i) = HiofKsta(:,i);
                        for k = 1:52-2
                            Hupdate(k+1,i) = (Hupdate(k+1,i)+ Hupdate(k,i) + Hupdate(k+2,i))/3;
                        end
                        HCDPiofKsta(:,i) =  (Hupdate(:,i)+ HCDPiofKsta0)/2;
                        Hsta(:,i) = HCDPiofKsta(dataInd,i,r);
                        SRsta(:,i,r) = ShatRiofKsta(dataInd,i,r);
                        SRazr(:,i,r) = ShatRiofKsta(dataInd,i,r);
                        ShatTiofK(:,i,r) = ShatRiofK(:,i,r)./chanEstCDPData(:,1,1);
                        DemappedRx(:,i) = wlan.internal.wlanConstellationDemodulate(ShatTiofK(:,i,r),mcsTable.NBPSCS,1);
                        DemappedTx(:,i) = wlan.internal.wlanConstellationDemodulate(ShatTiofKK(:,i,r),mcsTable.NBPSCS,1);
                        XT(:,i) = demapper( DemappedTx(:,i) ,Nsc);
                        X(:,i) = demapper( DemappedRx(:,i) ,Nsc);
                        HiofK(:,i) = ShatRiofK(:,i,r)./ X(:,i);
                        HiofKoriginal(:,i) = ShatRiofK(:,i,r)./ XT(:,i);
                        SCiofK0(:,1,r) =  SU./(HiofK(:,i));
                        for k =1:Nsc
                            if (SCiofK0(k,1,r)>0)
                                
                                XSC0(k,1) = 1;
                            else
                                XSC0(k,1) = -1;
                            end
                        end
                        if XSC0(:,1)==XSSC0(:,1)
                            
                            HCDPiofK(:,i) = (HiofK(:,i));
                            Hazr(:,i) = Hsta(:,i);
                        else
                            HCDPiofK(:,i) = chanEstCDPData(:,1,1);
                            Hazr(:,i) = HCDPiofK(:,i);
                        end
                        
                        
                    end
                    
                    
                    if i>1
                        
                        ShatTiofKsta(:,i,r) = ShatRiofKsta(:,i,r)./HCDPiofKsta(:,i-1);
                        ShatTiofKstadata(:,i,r) = ShatTiofKsta(dataInd,i,r);
                        ShatTiofKstapilot(:,i,r) = ShatTiofKsta(pilotInd,i,r);
                        DemappedRxstadata(:,i) = wlan.internal.wlanConstellationDemodulate(ShatTiofKstadata(:,i,r),mcsTable.NBPSCS,1);
                        DemappedRxstapilot(:,i) = wlan.internal.wlanConstellationDemodulate(ShatTiofKstapilot(:,i,r),2,1);
                        Xstadata(:,i) = demapper( DemappedRxstadata(:,i) ,Nsc);
                        Xstapilot(:,i) = demapper( DemappedRxstapilot(:,i),Np );
                        ShatRiofKstatemp(dataInd,i,r) = Xstadata(:,i);
                        ShatRiofKstatemp(pilotInd,i,r) = Xstapilot(:,i);
                        HiofKsta(:,i) = ShatRiofKsta(:,i,r)./ ShatRiofKstatemp(:,i);
                        Hupdate(:,i) = HiofKsta(:,i);
                        HupdateAzr(:,i) = HiofKsta(:,i);
                        for k = 1:52-2
                            Hupdate(k+1,i) = (Hupdate(k+1,i)+ Hupdate(k,i) + Hupdate(k+2,i))/3;
                        end
                        HCDPiofKsta(:,i) =  (Hupdate(:,i)+ HCDPiofKsta(:,i-1))/2;
                        Hsta(:,i) = HCDPiofKsta(dataInd,i,r);
                        SRsta(:,i,r) = ShatRiofKsta(dataInd,i,r);
                        SRazr(:,i,r) = ShatRiofKsta(dataInd,i,r);
                        ShatTiofK(:,i,r) = ShatRiofK(:,i,r)./HCDPiofK(:,i-1);
                        DemappedRx(:,i) = wlan.internal.wlanConstellationDemodulate(ShatTiofK(:,i,r),mcsTable.NBPSCS,1);
                        X(:,i) = demapper( DemappedRx(:,i) ,Nsc);
                        DemappedTx(:,i) = wlan.internal.wlanConstellationDemodulate(ShatTiofKK(:,i,r),mcsTable.NBPSCS,1);
                        XT(:,i) = demapper( DemappedTx(:,i),Nsc );
                        HiofK(:,i) = ShatRiofK(:,i,r)./ X(:,i);
                        HiofKoriginal(:,i) = ShatRiofK(:,i,r)./ XT(:,i);
                        SCiofK(:,i-1,r) = ShatRiofK(:,i-1,r)./(HiofK(:,i));
                        SSCiofK(:,i-1,r) = ShatRiofK(:,i-1,r)./(HCDPiofK(:,i-1));
                        DemappedSC(:,i-1) = wlan.internal.wlanConstellationDemodulate(SCiofK(:,i-1,r),mcsTable.NBPSCS,1);
                        DemappedSSC(:,i-1) = wlan.internal.wlanConstellationDemodulate(SSCiofK(:,i-1,r),mcsTable.NBPSCS,1);
                        XSC(:,i-1) = demapper( DemappedSC(:,i-1),Nsc);
                        XSSC(:,i-1) = demapper( DemappedSSC(:,i-1),Nsc );
                        if XSC(:,i-1)==XSSC(:,i-1)
                            HCDPiofK(:,i) = (HiofK(:,i));
                            Hazr(:,i) = (Hsta(:,i)+(HiofK(:,i)))/2;
                            jj(1,EsNodB (ii))=1;
                        else
                            
                            HCDPiofK(:,i) = HCDPiofK(:,i-1) ;
                            if i>1
                                Hazr(:,i) = HCDPiofKsta(dataInd,i-1,r); %Hazr(:,i-1) ;
                            end
                        end
                    end
                end
            end
            
        end
        
        dlmwrite('H:\Desktop Folders\Folders\MATLAB\HiofK.csv',HiofK,'delimiter',',','-append')
        [Np,Nsym,Nr] = size(rxPilots);
        % Calculate an estimate of the received pilots using the channel estimate
        temp = complex(zeros(Np,Nsym,Nsts,Nr));
        for r = 1:Nr
            for s = 1:Nsts
                for k = 1:Np
                    temp(k,:,s,r) = chanEstPilots(k,s,r).*refPilots(k,:,s);
                end
            end
        end
        % Sum over space-time streams and remove that dimension by permuting
        estRxPilots = permute(sum(temp,3),[1 2 4 3]);
        
        % Phase correction based on Allert val Zelst and Tim C. W. Schenk,
        % Implementation of a MIMO OFDM-Based Wireless LAN System, IEEE
        % Transactions on Signal Processing, Vol. 52, No. 2, February 2004. The
        % result is averaged over the number of receive antennas (summed over the
        % 3rd dimension).
        cpe = angle(sum(sum(rxPilots.*conj(estRxPilots),1),3));
        sym = ShatRiofK;%ofdmDemodData;
        [Nsc,~,Nr] = size(sym);
        x = exp(-1i*cpe); % Create constant
        ofdmDemodData = ShatRiofK;
        [rxBits_sim,csi_sim(:,:)] = recoverBitsNow( ofdmDemodData,Nsym,Nr,Nsts,HCDPiofK,noiseVarEst,mcsTable,pNcbpssi,cfgNHT10);
        [rxBits_simsta,csi_simsta(:,:) ]= recoverBitsNow( ofdmDemodData,Nsym,Nr,Nsts,Hsta,noiseVarEst,mcsTable,pNcbpssi,cfgNHT10);
        [rxBits_simazr,csi_simazr(:,:) ]= recoverBitsNow( ofdmDemodData,Nsym,Nr,Nsts,Hazr,noiseVarEst,mcsTable,pNcbpssi,cfgNHT10);
        HhatCDP=zeros(64,1);
        HhatCDP2=HhatCDP;
        for xu=2:63
            HhatCDP(xu,1)=( HhatLS(xu,1) + HhatLS(xu+1,1)+ HhatLS(xu-1,1))/3;
        end
        HhatCDP = 0.5*(HhatCDP+HhatDFT);
        rxBits_CDP = wlanNonHTDataRecover(rx(ind.NonHTData(1):ind.NonHTData(2),:), HhatCDP(7:58,1), noiseVarEst, cfgNHT10, cfgRec);
        [numerr_CDP, ber_CDP] = biterr(rxBits_sim, inpPSDU);
        [numerr_STA, ber_STA] = biterr(rxBits_simsta, inpPSDU); % Compare bits
        [numerr_azr, ber_azr] = biterr(rxBits_simazr, inpPSDU); % Compare bits
        [numerr_LS, ber_LS] = biterr(rxBits_LS, inpPSDU); % Compare bits
        [numerr_LDFT, ber_LDFT] = biterr(rxBits_LDFT, inpPSDU); % Compare bits
        
        % Compare bits
        NumOfErrorBitCDP(1,mc) =   ber_CDP;
        NumOfErrorBitazr(1,mc) =   ber_azr;
        NumOfErrorBitSTA(1,mc) =   ber_STA;
        NumOfErrorBitLS(1,mc) =   ber_LS;
        NumOfErrorBitLDFT(1,mc) =   ber_LDFT;
        HOFLS = HiofK;
        HOFDFT = HiofK;
        HOFCDP = HCDPiofK;
        HOFSTA = Hsta;
        HOFAZR = Hazr;
        TEMPHDFT =  HhatDFT(7:58,1);
        for pin=1:Nsym
            HOFLS(:,pin) = HhatLSo(dataInd,:,:);
            HOFDFT(:,pin) = TEMPHDFT( dataInd,:,:);
        end
        HOFLS = ifft(HOFLS);
        HOFDFT = ifft( HOFDFT);
        HOFCDP = ifft( HOFCDP);
        HOFSTA = ifft( HOFSTA);
        HOFAZR =  ifft(HOFAZR);
        HiofKoriginal =ifft(HiofKoriginal);
        ChMSE_LSo = ChMSE_LSo +sqrt( mse(abs(HOFLS-HiofKoriginal)));
        ChMSE_DFT = ChMSE_DFT +  sqrt( mse(abs(HOFDFT-HiofKoriginal)));
        ChMSE_CDP = ChMSE_CDP  +  sqrt(  mse(abs(HOFCDP-HiofKoriginal)));
        ChMSE_STA = ChMSE_STA +   sqrt( mse(abs(HOFSTA-HiofKoriginal)));
        ChMSE_iCDP = ChMSE_iCDP  +    sqrt( mse(abs(HOFAZR-HiofKoriginal)));
    end
    BERSTA(1, (ii)) =  mean(NumOfErrorBitSTA);
    BERDFT(1, (ii)) =   mean(NumOfErrorBitLDFT);
    BERLS(1, (ii))    = mean(NumOfErrorBitLS);
    BERiCDP(1, (ii))    = mean(NumOfErrorBitazr);
    BERCDP(1, (ii))    = mean(NumOfErrorBitCDP);
    NumOfErrorBitLS    = zeros(1,MC);
    NumOfErrorBitCDP    = zeros(1,MC);
    NumOfErrorBitLDFT    = zeros(1,MC);
    NumOfErrorBitSTA    = zeros(1,MC);
    NumOfErrorBitazr = zeros(1,MC);
    MSErrorLS(1,ii) = ChMSE_LSo/MC;
    MSErrorDFT(1,ii) = ChMSE_DFT/MC;
    MSErrorCDP(1,ii) = ChMSE_CDP/MC;
    MSErrorSTA(1,ii) = ChMSE_STA/MC;
    MSErroriCDP(1,ii) = ChMSE_iCDP/MC;
end
semilogy(EsNodB,BERLS,'bx-','LineWidth',1.2); hold on
semilogy(EsNodB,BERDFT,'cs-','LineWidth',1.2);hold on
semilogy(EsNodB,BERCDP,'mo-','LineWidth',1.2);hold on
semilogy(EsNodB,BERSTA,'rp-','LineWidth',1.2);hold on
semilogy(EsNodB,BERiCDP,'gd-','LineWidth',1.2);hold on
title('HyperLAN Channel Model "E"')
legend('LS','DFT','CDP','STA','iCDP');
%axis([1 40 1e-4 1 ])
xlabel('SNR(dB)')
ylabel('BER')
grid on ;
figure
plot(EsNodB,MSErrorLS,'bx-'); hold on
plot(EsNodB,MSErrorDFT,'go-');hold on
plot(EsNodB,MSErrorCDP,'mo-');hold on
plot(EsNodB,MSErrorSTA,'yo-');hold on
plot(EsNodB,MSErroriCDP,'ro-');hold on

title('HyperLAN Channel Model "E"')

legend('LS','DFT','CDP','STA','iCDP');

xlabel('SNR(dB)')
ylabel('RMSE')
grid on ;














