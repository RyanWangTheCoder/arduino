%% Step1 : Read Data from .txt tile_Pre
close all;
gsr_all = load('gsr_10mins_matlab.txt');
N = 5000;
gsr = gsr_all(1:(1+N), 2);
xden = wdenoise(gsr);  %% https://www.mathworks.com/help/wavelet/ref/wdenoise.html
plot([gsr' xden'])
legend("Original Signal","Denoised Signal")