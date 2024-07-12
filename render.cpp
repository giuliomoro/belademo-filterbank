#include <Bela.h>
#include <vector>
#include <libraries/Biquad/QuadBiquad.h>
#include <libraries/Trill/Trill.h>
#include <libraries/AudioFile/AudioFile.h>
#include <stdlib.h>
#include <math.h>
#include <libraries/DelayLine/DelayLine.h>
#include <libraries/OnePole/OnePole.h>
#include <libraries/Scope/Scope.h>
#include <libraries/math_neon/math_neon.h>
Scope scope;
OnePole feedbackSmoother;
OnePole timeSmoother;
OnePole mixSmoother;
std::vector<DelayLine> delays;
float gMaxDelay = 1001; // Maximum delay time (ms)
float gDelayTime = 420; // Delay Time (ms)
float gWetDryMix = 0.5; // Wet/Dry Mix
float gFeedback = 0; // Feedback
uint64_t lastFrames;
uint64_t lastUpdatedDelay;

std::vector<QuadBiquad*> bs;
static constexpr size_t datasz = 4;
Trill trillFilter;
Trill trillVolume;
Trill trillMix;
Trill trillDelay;
AudioFileReader player;

void readTrill(void*)
{
	while(!Bela_stopRequested())
	{
		trillMix.readI2C();
		trillFilter.readI2C();
		trillVolume.readI2C();
		trillDelay.readI2C();
		if(trillDelay.getNumTouches())
		{
			float in = 0.2f * trillDelay.compoundTouchSize();
			gFeedback = constrain(map(in, 0, 1, 0.1
			, 1.1), 0.1, 1.1);
			float val = trillDelay.compoundTouchLocation();
			val *= val;
			gDelayTime = map(val, 0, 1, 100, gMaxDelay - 10);
			// rt_printf("to: %f gF: %f gD: %f\n", in, gFeedback, gDelayTime);
			gWetDryMix = 0.5f;
			lastUpdatedDelay = lastFrames;
		}
		usleep(4000);
	}
}

std::vector<AudioFileReader> gReaders(2);
std::vector<std::string> gFilenames = {
	"Rebuscona.wav",
	"Rebuscona.wav",
};
size_t gLoadingFile;
size_t gCurrReader;

std::vector<float> gSamples;
size_t gFrameCount = 0;
AuxiliaryTask gStartLoadingFileTask;

void loadNextFile(void*)
{
	// start preloading the next file
	gLoadingFile = (gLoadingFile + 1) % gFilenames.size();
	size_t nextReader = (gCurrReader + 1) % gReaders.size();
	gReaders[nextReader].setup(gFilenames[gLoadingFile], 16384);
	rt_printf("Opening file [%zu] %s in reader %zu\n", gLoadingFile, gFilenames[gLoadingFile].c_str(), nextReader);
}

bool setup(BelaContext *context, void *userData)
{
	scope.setup(2, context->audioSampleRate);
	mixSmoother.setup(50, context->audioSampleRate);
	feedbackSmoother.setup(5, context->audioSampleRate);
	timeSmoother.setup(5, context->audioSampleRate);
	size_t channels = std::min(context->audioOutChannels, datasz / 2);
	delays.resize(channels);
	for(auto& d : delays)
	{
		int ret;
		if((ret = d.setup(gMaxDelay + 1, context->audioSampleRate, 1)))
		{
			printf("Error setting up delay. Error code: %d/n", ret);
			return false;
		}
		d.useSeparateTaps(false);
		d.setWetDryMix(gWetDryMix);
		d.setDelayTime(gDelayTime);
		d.setFeedbackGain(gFeedback);
	}
	if(0 == trillFilter.setup(1, Trill::CRAFT, 0x37))
		printf("CRAFT OK\n");
	usleep(10000);
	trillFilter.setMode(Trill::DIFF);
	usleep(10000);
	trillFilter.setPrescaler(2);
	
	if(0 == trillMix.setup(1, Trill::RING, 0x3e))
	{
		printf("RING OK\n");
		usleep(10000);
		trillVolume.setNoiseThreshold(0.1);
	}
	if(0 == trillVolume.setup(1, Trill::BAR, 0x20))
	{
		printf("BAR OK\n");
		usleep(10000);
		trillVolume.setNoiseThreshold(0.1);
	}
	if(0 == trillDelay.setup(1, Trill::FLEX))
	{
		printf("FLEX OK\n");
		usleep(10000);
		trillDelay.setPrescaler(3);
		usleep(10000);
		trillDelay.setNoiseThreshold(0.1);
		usleep(10000);
		trillDelay.updateBaseline();
	}
	Bela_runAuxiliaryTask(readTrill);

	gStartLoadingFileTask = Bela_createAuxiliaryTask(loadNextFile, 1, "loadNextFile");
	if(!gStartLoadingFileTask) {
		fprintf(stderr, "Error creating file loading task\n");
		return false;
	}
	gLoadingFile = -1;
	gCurrReader = -1;
	// open the first file
	loadNextFile(NULL);
	gCurrReader = 0;
	// open the second file
	loadNextFile(NULL);
	gSamples.reserve(context->audioFrames * gReaders[gCurrReader].getChannels());

	bs.resize(trillFilter.getNumChannels() * 2);
	BiquadCoeff::Settings s = {
		.fs = context->audioSampleRate,
		.q = 0.8,
		.peakGainDb = 0,
	};
	// Setting cutoff and type for all the filters
	float cutoff = 50;
	for(unsigned int b = 0; b < bs.size(); b += 2)
	{
		printf("%.2f ", cutoff);
		for(size_t n = 0; n < 2; ++n)
		{
			s.cutoff = cutoff;
			s.type = 0 == n ? BiquadCoeff::highpass :  BiquadCoeff::lowpass;
			bs[b + n] = new QuadBiquad;
			unsigned int q;
			// half of the filters for processing audio
			for(q = 0; q < datasz / 2; ++q)
				bs[b + n]->filters[q].setup(s);
			// the other half to process amplitude coefficients
			// these are coming from Trill readings, so we smooth them out a bti
			for(q = datasz / 2; q < datasz; ++q)
			{
				s.cutoff = 100;
				s.type = BiquadCoeff::lowpass;
				bs[b + n]->filters[q].setup(s);
			}
			bs[b + n]->update();
		}
		// 150 -> 1.08
		// 100 -> 1.13
		cutoff *= 1.13;
	}
	printf("\n");
	return true;
}

#include <algorithm>

void render(BelaContext *context, void *userData)
{
	lastFrames = context->audioFramesElapsed;
	AudioFileReader& reader = gReaders[gCurrReader];
	// this call may allocate memory if getChannels() is larger than for
	// all previous files
	gSamples.resize(context->audioFrames * reader.getChannels());
	reader.getSamples(gSamples);
	size_t channels = std::min(std::min(context->audioOutChannels, datasz / 2), reader.getChannels());
	static float coeff = 2;
	static float mix = 1;
	if(trillMix.getNumTouches())
	{
		float val = trillMix.compoundTouchLocation();
		// two dead zones, symmetrical linear crossfading in between
		if(val > 0.5)
			val = 1.f - val;
		val *= 2.f;
		val = map(val, 0.2, 0.8, 0, 1);
		val = constrain(val, 0, 1);
		mix = val;
	}
	if(trillVolume.getNumTouches())
	{
		coeff = constrain(trillVolume.compoundTouchLocation(), 0, 0.999);
		coeff *= coeff;
		coeff *= 3.f;
	}
	for(unsigned int n = 0; n < context->audioFrames; ++n)
	{
		std::array<float,datasz> data;
		
		// get an input from player
		float ins[channels];
		float in = (rand() / float(RAND_MAX) - 0.5f) * 2.5;
		for(size_t c = 0; c < channels; ++c)
			ins[c] = (1.f - mix) * in + mix * gSamples[n * reader.getChannels() + c];
		// count samples played back for the existing file
		gFrameCount++;
		if(gFrameCount >= reader.getLength())
		{
			// reached end of file
			gFrameCount = 0;
			// start playing the file we preloaded
			gCurrReader = (gCurrReader + 1) % gReaders.size();
			reader.getSamples(gSamples);
			rt_printf("Playing from reader: %zu\n", gCurrReader);
			// start loading next file in a real-time safe way
			Bela_scheduleAuxiliaryTask(gStartLoadingFileTask);
		}

		// process
		std::array<float,datasz> outs;
		outs.fill(0);
		for(size_t b = 0; b < bs.size(); b += 2)
		{
			size_t q;
			for(q = 0 ; q < channels; ++q)
				data[q] = ins[q];
			float mul = coeff * trillFilter.rawData[b / float(bs.size()) * trillFilter.getNumChannels()];
			mul *= mul;
			for(q = datasz / 2; q < datasz; ++q)
				data[q] = 0.01f + mul;
			for(size_t s = 0; s < 2; ++s)
			{
				// use the same data for both filters: process in series 
				bs[b + s]->process(data.data());
			}
			for(size_t c = 0; c < channels; ++c)
				outs[c] += data[c] * data[c + datasz / 2];
		}
		// apply delay
		for(auto& d : delays)
		{
			d.setFeedbackGain(feedbackSmoother.process(gFeedback));
			d.setDelayTime(timeSmoother.process(gDelayTime));
			d.setWetDryMix(mixSmoother.process(gWetDryMix));
			gFeedback *= 0.99994;
			gWetDryMix *= 0.999993;
		}
		float r[2];
		for(unsigned int c = 0; c < channels; ++c)
		{
			r[c] = tanhf_neon(delays[c].process(outs[c]));
			audioWrite(context, n, c, r[c]);
		}
		scope.log(r);
	}
}

void cleanup(BelaContext *context, void *userData)
{
	for(auto& b : bs)
		delete b;
}
