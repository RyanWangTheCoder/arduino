import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from scipy import signal, datasets
from scipy.signal import butter, lfilter, freqz
from datetime import datetime
from pykalman import KalmanFilter
from statsmodels.tsa.holtwinters import ExponentialSmoothing
from scipy.signal import savgol_filter
import pylab
#from scipy.signal import complex_cepstrum
from scipy.fft import fftshift

class EMGDataHelper:
    def __init__(self, data_file):
        self.dataFile = data_file
        self.data = pd.read_csv(self.dataFile, sep=' --> ', header=None, names=["time", "value"], engine='python')
        print(self.data)
        for index, row in self.data.iterrows():
            t = self.data.loc[index, 'time']
            if index == 0:
                start_log_time = datetime.strptime(t, '%H:%M:%S.%f')
            duration = datetime.strptime(t, '%H:%M:%S.%f') - start_log_time
            self.data.iloc[index, 0] = duration.seconds * 1000 + duration.microseconds/1000
            self.data.iloc[index, 1] = float(self.data.loc[index, 'value'])
        print(self.data)
        self.x = np.linspace(1, len(self.data), num=len(self.data))
        self.timeX = self.data['time']
        self.dataValue = self.data['value']

        time_series = self.data['time']
        end_time = time_series[len(time_series) - 1]
        self.timestamps = np.linspace(0, end_time, len(time_series))
# reading the database
    def doInterp(self, dfData):
        dataValue = dfData['value']
        x = np.linspace(1, len(dfData), num=len(dfData))
        time_series = dfData['time']
        end_time = time_series[len(time_series) - 1]
        timestamps = np.linspace(0, end_time, len(time_series))

        spl = CubicSpline(x, dataValue)

        fig, ax = plt.subplots(4, 1)
        plt.grid()

        # Scatter plot with day against tip
        #ax[0].plot(self.x, spl(self.x))
        ax[0].plot(timestamps/1000, dataValue, 'o', label='data')
        plt.title("EMG Plot")
        ax[1].plot(timestamps/1000, spl(x, nu=1), '--', label='1st derivative')
        ax[2].plot(timestamps/1000, spl(x, nu=2), '--', label='2nd derivative')
        ax[3].plot(timestamps/1000, spl(x, nu=3), '--', label='3rd derivative')
        # Adding Title to the Plot
        ax[0].title.set_text('EMG Plot')
        ax[1].title.set_text('1st derivative')
        ax[2].title.set_text('2nd derivative')
        ax[3].title.set_text('3rd derivative')
        plt.xlabel('Time [sec]')
        for ax_idx in ax.flat:
            ax_idx.grid(True)
        # Setting the X and Y labels
        #plt.xlabel('Day')
        #plt.ylabel('Tip')
        #plt.legend()

        plt.subplots_adjust(hspace=0.35)
        plt.show()

        return

    def butter_lowpass(self, cutoff, fs, order=5):
        return butter(order, cutoff, fs=fs, btype='low', analog=False)

    def butter_lowpass_filter(self, data, cutoff, fs, order=5):
        b, a = self.butter_lowpass(cutoff, fs, order=order)
        y = lfilter(b, a, data)
        return y
    def doMovingAverage(self, dfData):
        dataValue = dfData['value']
        x = np.linspace(1, len(dataValue), num=len(dataValue))
        dfData['time'] = dfData['time'] - dfData['time'].iloc[0]
        time_series = dfData['time']
        #print(time_series, time_series.iloc[0])
        end_time = time_series.iloc[len(time_series) - 1]
        #print(end_time)
        timestamps = np.linspace(0, end_time, len(time_series))

        # Filter requirements.
        order = 6
        fs = 300.0  # sample rate, Hz
        cutoff = 3.667  # desired cutoff frequency of the filter, Hz

        # Get the filter coefficients so we can check its frequency response.
        b, a = self.butter_lowpass(cutoff, fs, order)

        # Plot the frequency response.
        w, h = freqz(b, a, fs=fs, worN=8000)
        plt.plot(w, np.abs(h), 'b')
        plt.plot(cutoff, 0.5 * np.sqrt(2), 'ko')
        plt.axvline(cutoff, color='k')
        plt.xlim(0, 0.5 * fs)
        plt.title("Lowpass Filter Frequency Response")
        plt.xlabel('Frequency [Hz]')
        plt.grid()
        plt.figure()
        # Demonstrate the use of the filter.
        # First make some data to be filtered.
        # Filter the data, and plot both the original and filtered signals.
        y = self.butter_lowpass_filter(dataValue, cutoff, fs, order)


        plt.subplot(2, 1, 1)
        plt.plot(timestamps/1000, dataValue, 'b-', label='data')
        #plt.plot(self.x, y, 'g-', linewidth=2, label='filtered data')
        plt.xlabel('Time [sec]')
        plt.ylabel('GSR data')

        plt.grid()

        plt.subplot(2, 1, 2)
        plt.plot(timestamps / 1000, dataValue.rolling(window=50).mean(), 'b-', label='data')
        # plt.plot(self.x, y, 'g-', linewidth=2, label='filtered data')
        plt.xlabel('Time [sec]')
        plt.ylabel('GSR data')

        plt.grid()

        plt.subplots_adjust(hspace=0.35)
        plt.show()

    def doKalmanFilter(self, dfData):
        dataValue = dfData['value']
        x = np.linspace(1, len(dataValue), num=len(dataValue))
        dfData['time'] = dfData['time'] - dfData['time'].iloc[0]
        time_series = dfData['time']
        #print(time_series, time_series.iloc[0])
        end_time = time_series.iloc[len(time_series) - 1]
        #print(end_time)
        timestamps = np.linspace(0, end_time, len(time_series))

        # Define Kalman filter model
        kf = KalmanFilter(transition_matrices=[1],
                          observation_matrices=[1],
                          initial_state_mean=dfData['value'].iloc[0],
                          initial_state_covariance=1,
                          observation_covariance=1,
                          transition_covariance=0.01)

        # Fit model and make predictions
        state_means, _ = kf.filter(dataValue)
        state_means = state_means.flatten()

        # Plot original data and Kalman filter predictions
        plt.plot(timestamps / 1000, dataValue, label='Original Data')
        plt.plot(timestamps / 1000, state_means, label='Kalman Filter Predictions')
        plt.xlabel('Time [sec]')
        plt.ylabel('EMG data')

        plt.legend()
        plt.grid()

        plt.show()
    def doSavitzkyGolayFilter(self, dfData):
        dataValue = dfData['value']
        x = np.linspace(1, len(dataValue), num=len(dataValue))
        dfData['time'] = dfData['time'] - dfData['time'].iloc[0]
        time_series = dfData['time']
        #print(time_series, time_series.iloc[0])
        end_time = time_series.iloc[len(time_series) - 1]
        #print(end_time)
        timestamps = np.linspace(0, end_time, len(time_series))

        # Calculate smoothed values with Savitzky-Golay method
        smoothed = savgol_filter(dataValue, window_length=50, polyorder=5)

        # Plot original data and smoothed values
        plt.plot(timestamps / 1000, dataValue, label='Original Data')
        plt.plot(timestamps / 1000, smoothed, label='Smoothed Values')
        plt.legend()
        plt.grid()
        plt.show()
    def exponentialFilter(self, dfData):
        dataValue = dfData['value']
        x = np.linspace(1, len(dataValue), num=len(dataValue))
        dfData['time'] = dfData['time'] - dfData['time'].iloc[0]
        time_series = dfData['time']
        #print(time_series, time_series.iloc[0])
        end_time = time_series.iloc[len(time_series) - 1]
        #print(end_time)
        timestamps = np.linspace(0, end_time, len(time_series))

        # fit the model to the data
        fit_model = model.fit()

        # generate smoothed values for the next 12 months
        smoothed_values = fit_model.predict(start=len(dataValue), end=len(dataValue) + 11)
        plt.plot(smoothed_values, label='Exponential Smoothing')
        plt.legend()
        plt.grid()
        plt.show()

    def doSpectrum(self, dfData):
        dataValue = dfData['value']
        x = np.linspace(1, len(dataValue), num=len(dataValue))
        dfData['time'] = dfData['time'] - dfData['time'].iloc[0]
        time_series = dfData['time']
        # print(time_series, time_series.iloc[0])
        end_time = time_series.iloc[len(time_series) - 1]
        # print(end_time)
        timestamps = np.linspace(0, end_time, len(time_series))

        fig, ax = plt.subplots(2, figsize=(8, 7))

        fs = 64
        f, t, Sxx = signal.spectrogram(dataValue, fs)
        ax[0].pcolormesh(t, f, Sxx)
        ax[0].set_ylabel('Frequency [Hz]')
        ax[0].set_xlabel('Time [sec]')

        # Note, if using output that is not one sided, then use the following:
        f, t, Sxx = signal.spectrogram(dataValue, fs, return_onesided=False)
        ax[1].pcolormesh(t, fftshift(f), fftshift(Sxx, axes=0))
        ax[1].set_ylabel('Frequency [Hz]')
        ax[1].set_xlabel('Time [sec]')
        plt.show()

    def doFreqPlot(self, dfData):
        dataValue = dfData['value']
        x = np.linspace(1, len(dataValue), num=len(dataValue))
        dfData['time'] = dfData['time'] - dfData['time'].iloc[0]
        time_series = dfData['time']
        #print(time_series, time_series.iloc[0])
        end_time = time_series.iloc[len(time_series) - 1]
        #print(end_time)
        timestamps = np.linspace(0, end_time, len(time_series))

        #FFT = abs(scipy.fft(self.dataValue))
        FFT = np.fft.fft(dataValue)
        #freqs = scipy.fftpack.fftfreq(len(self.timestamp), self.timestamps[1] - self.timestamps[0])

        pylab.subplot(211)
        pylab.plot(timestamps/1000, dataValue)
        pylab.grid()
        pylab.subplot(212)
        pylab.plot(timestamps/1000, 20 * np.log10(FFT), '.')
        pylab.grid()
        pylab.show()

    def doTwoSpectrum(self, dfData, dfData2):
        dataValue = dfData['value']
        x = np.linspace(1, len(dataValue), num=len(dataValue))
        dfData['time'] = dfData['time'] - dfData['time'].iloc[0]
        time_series = dfData['time']
        # print(time_series, time_series.iloc[0])
        end_time = time_series.iloc[len(time_series) - 1]
        # print(end_time)
        timestamps = np.linspace(0, end_time, len(time_series))

        dataValue2 = dfData2['value']

        fig, ax = plt.subplots(2, figsize=(8, 7))

        fs = 64
        f, t, Sxx = signal.spectrogram(dataValue, fs, return_onesided=False)
        ax[0].pcolormesh(t, fftshift(f), fftshift(Sxx, axes=0))
        ax[0].set_ylabel('Frequency [Hz]')
        ax[0].set_xlabel('Time [sec]')

        # Note, if using output that is not one sided, then use the following:
        f, t, Sxx = signal.spectrogram(dataValue2, fs, return_onesided=False)
        ax[1].pcolormesh(t, fftshift(f), fftshift(Sxx, axes=0))
        ax[1].set_ylabel('Frequency [Hz]')
        ax[1].set_xlabel('Time [sec]')
        plt.show()
    def splitHeadTail(self):
        subFirst50 = self.data[(self.data["time"] > 150000)]
        subFirst50 = subFirst50[(subFirst50["time"] < 200000)]
        subLast50 = self.data[(self.data["time"] > 380000)]
        subLast50 = subLast50[(subLast50["time"] < 430000)]
        print(subFirst50)
        self.doMovingAverage(subFirst50)
        #self.doFreqPlot(subFirst50)
        #print(subLast50)
        self.doMovingAverage(subLast50)
        #self.doFreqPlot(subLast50)
        self.doTwoSpectrum(subFirst50, subLast50)
class GSRDataHelper:
    def __init__(self, data_file):
        self.dataFile = data_file
        self.data = pd.read_csv(self.dataFile, sep=' --> ', header=None, names=["time", "value"], engine='python')
        print(self.data)
        for index, row in self.data.iterrows():
            t = self.data.loc[index, 'time']
            self.data.iloc[index, 0] = datetime.strptime(t, '%H:%M:%S.%f').time()
            self.data.iloc[index, 1] = float(self.data.loc[index, 'value'])
        print(self.data)


if __name__ == '__main__':
    emgData = EMGDataHelper("C:\\Users\\ryanw\\OneDrive\\Documents\\Arduino\\emg.txt")
    emgData.doInterp(emgData.data)
    emgData.doMovingAverage(emgData.data)
    emgData.doKalmanFilter(emgData.data)
    #emgData.doSavitzkyGolayFilter(emgData.data)
    #emgData.doFreqPlot(emgData.data)
    #emgData.exponentialFilter(emgData.data)
    #gsrData = GSRDataHelper("C:\\Users\\ryanw\\OneDrive\\Documents\\Arduino\\new.txt")
    #emgData.splitHeadTail()