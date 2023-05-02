clear all
close all
clc

fc=10;%frequency of the carrier
fs=32*fc;%sampling frequency with oversampling factor=32
t=0:1/fs:2-1/fs;% 2 seconds duration
x=cos(2*pi*fc*t);% time domain signal (real number)
subplot(3,1,1); plot(t,x);hold on; % plot the signal
title('x[n]=cos(2 \pi 10 t)'); xlabel('n'); ylabel('x[n]');

N=256; %FFT size
X = fft(x,N);%N-point complex DFT
%output contains DC at index 1, Nyquist frequency at N/2+1 th index
%positive frequencies from index 2 to N/2
%negative frequencies from index N/2+1 to N

X(1)
abs(X(8:10)) %display samples 7 to 9



%calculate frequency bins with FFT
df=fs/N %frequency resolution
sampleIndex = 0:N-1; %raw index for FFT plot
f=sampleIndex*df; %x-axis index converted to frequencies


subplot(3,1,2); stem(sampleIndex,abs(X)); %sample values on x-axis
title('X[k]'); xlabel('k'); ylabel('|X(k)|');
subplot(3,1,3); plot(f,abs(X)); %x-axis represent frequencies
title('X[k]'); xlabel('frequencies (f)'); ylabel('|X(k)|');

%two-sided FFT with negative frequencies on left and positive frequencies on right
%following works only if N is even, for odd N see equation above
X1 = [(X(N/2+1:N)) X(1:N/2)]; %order frequencies without using fftShift
X2 = fftshift(X);%order frequencies by using fftshift

df=fs/N; %frequency resolution
sampleIndex = -N/2:N/2-1; %raw index for FFT plot
f=sampleIndex*df; %x-axis index converted to frequencies
%plot ordered spectrum using the two methods
figure;subplot(2,1,1);stem(sampleIndex,abs(X1));hold on; stem(sampleIndex,abs(X2),'r') %sample index on x-axis
title('Frequency Domain'); xlabel('k'); ylabel('|X(k)|');%x-axis represent sample index
subplot(2,1,2);stem(f,abs(X1)); stem(f,abs(X2),'r') %x-axis represent frequencies
xlabel('frequencies (f)'); ylabel('|X(k)|');
