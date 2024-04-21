%% Step1 : Read Data from .txt tile_Pre
close all;
threshold = 648.3;

d1 = gsr_util('gsr001matlab.txt', threshold, 'before activity');
d2 = gsr_util('gsr002matlab.txt', threshold, 'during break');
d3 = gsr_util('gsr003matlab.txt', threshold, 'after activity');
movmean_plot(cat(1, d1, d2, d3), 'Moving average of all 3 phases');

function movmean_plot(data, title_txt)
    figure;
    t = movmean(data, 100);
    plot((1:size(t))/10, t, '');
    xlabel('seconds');
    ylabel('skin resistance');
    title(title_txt);
    grid on;
end
function gsr = gsr_util(file, threshold, title_txt)
    gsr_all = load(file);
    %peaks = findpeaks(gsr_all(1:size(gsr_all, 1), 2));
    N = 200;
    gsr = gsr_all(1:(1+N), 2);
    peaks = findpeaks(gsr);
    size(peaks)
    xden = wdenoise(gsr);  %% https://www.mathworks.com/help/wavelet/ref/wdenoise.html
    figure;
    subplot(2,1,1);    
    plot((1:N)/10, gsr(1:N), '', (1:N)/10, xden(1:N), '')
    xlabel('seconds') 
    ylabel('skin resistance') 
    title(title_txt);
    grid on;
    subplot(2,1,2);
    t = movmean(gsr, 100);
    plot((1:size(t))/10, t, '')
    xlabel('seconds');
    ylabel('skin resistance');
    legend("Original Signal","Denoised Signal");
    ylim([640 650])
    peak_count = sum(gsr>threshold);
    mean(gsr)
    grid on
end
    