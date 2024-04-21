%% Step1 : Read Data from .txt tile_Pre
close all;
data = load('emg_first_last_matlab.txt');

N = 2048*2;
emg1 = emg(7500:(7500+N), 2);
emg2 = emg((27969-N):27969, 2);

emg_util(emg1, 'first');
emg_util(emg2, 'last');


function emg = emg_util(data, title_txt)
    Fe=10; %Samling frequenc
    Fc=3; % Cut-off frequency (from 2 Hz to 6 Hz depending to the type of your electrod)
    Nc=4; % Filter Order
    [B, A] = butter(Nc,Fc*2/Fe, 'low'); %filter's parameters
    filtered_data=filtfilt(B, A, data); %in the case of Off-line treatment
    
    figure;
    subplot(2,1,1);    
    plot((1:size(data))/10, data, '', (1:size(data))/10, filtered_data, '')
    xlabel('seconds') 
    ylabel('EMG') 
    legend("Original Signal","Denoised Signal");
    %ylim([640 650])
    title(title_txt);
    grid on;
    
    subplot(2,1,2);    
    t = movmean(data, 10);
    plot((1:size(t))/10, t, '')
    xlabel('seconds');
    ylabel('EMG');
    legend("Original Signal","Denoised Signal");
    grid on;
end