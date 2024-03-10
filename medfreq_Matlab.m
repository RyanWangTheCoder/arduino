%% Step1 : Read Data from .txt tile_Pre
close all;
fq = 100; %sampling frequency
emg = load('emg_first_last_matlab.txt');
N = 2048*2;
emg1 = emg(7500:(7500+N), 2);
emg2 = emg((27969-N):27969, 2);
title('first')
plot(emg1);
grid on;
figure;
plot(emg2);
title('last');
grid on;
figure;
disp('medfreq')
medfreq(emg1)
disp('rms')
rms(detrend(emg1,0))
figure;
medfreq(emg2)
rms(detrend(emg2,0))

h = spectrum.periodogram;

s = detrend(emg1,0);

Hpsd = psd(h,s,'Fs',fq,'NFFT',2*nextpow2(length(s)));
Pdist = cumsum(Hpsd.Data);
Freq = Hpsd.Frequencies;
OverHalfIdx = find(Pdist>=Pdist(end)/2,1,'first');
UnderHalfIdx = find(Pdist<=Pdist(end)/2,1,'last');
MidFreq = (Freq(OverHalfIdx)+Freq(UnderHalfIdx))/2


s = detrend(emg2,0);

Hpsd = psd(h,s,'Fs',fq,'NFFT',2*nextpow2(length(s)));
Pdist = cumsum(Hpsd.Data);
Freq = Hpsd.Frequencies;
OverHalfIdx = find(Pdist>=Pdist(end)/2,1,'first');
UnderHalfIdx = find(Pdist<=Pdist(end)/2,1,'last');
MidFreq = (Freq(OverHalfIdx)+Freq(UnderHalfIdx))/2
return;

Fs=1000;
t1=emg1;
x1=emg2;
figure;
plot(t1,x1);
xlabel('Time(s)'); ylabel('magnitude');
title('Pre Raw EMG');
%% step2: Perform FFT_Pre
L=length(x1);
Y=fft(x1);
P2=abs(Y/L);
B=P2(1:L/2+1);
B(2:end-1) = 2*B(2:end-1);
A=Fs*(0:(L/2))/L;
figure;
plot(A,B);
axis([10 450 0 0.02]);
xlabel('Frequency(Hz)');
ylabel('magnitude');
title('Pre FFT');