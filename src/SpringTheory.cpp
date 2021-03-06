#include "plugin.hpp"

const float MINIMUM_TIME = 1e-3f;
const float MAXIMUM_TIME = 10.f;
const float TIME_RATIO = MAXIMUM_TIME / MINIMUM_TIME;

const int NOTES = 12;
const int CENTS = 100;
const float UNNOTICEABLE_DIFFERENCE = 1.f / (NOTES * CENTS);
const float LOG_UNNOTICEABLE_DIFFERENCE = log(UNNOTICEABLE_DIFFERENCE);

Vec grid(float x, float y) {
	float halfGrid = RACK_GRID_WIDTH * 0.5f;
	return Vec(halfGrid * x, halfGrid * y);
}

struct SpringTheory : Module {
	enum ParamIds {
		OFFSET_PARAM,
		FREQUENCY_PARAM,
		ATTACK_PARAM,
		POSITION_PARAM,
		NUM_PARAMS
	};
	enum InputIds {
		FREQUENCY_INPUT,
		ATTACK_INPUT,
		POSITION_INPUT,
		RESET_INPUT,
		NUM_INPUTS
	};
	enum OutputIds {
		ACCELERATION_OUTPUT,
		VELOCITY_OUTPUT,
		POSITION_OUTPUT,
		NUM_OUTPUTS
	};
	enum LightIds {
		DIFFERENCE_LIGHT,
		NUM_LIGHTS
	};

	SpringTheory() {
		config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

		configParam(OFFSET_PARAM, 0.f, 1.f, 1.f, "Offset");
		configParam(FREQUENCY_PARAM, -5.f, 5.f, 0.f, "Frequency", "Hz", 2, 2);
		configParam(ATTACK_PARAM, 0.f, 1.f, 0.5f, "Attack", "ms", TIME_RATIO, MINIMUM_TIME * 1000);
		configParam(POSITION_PARAM, -5.f, 5.f, -5.f, "Position", "m");
	}

	dsp::SchmittTrigger resetTrigger;

    float velocity = 0.f;
	float position = 0.f;

	void process(const ProcessArgs& args) override {
		// read and normalize parameters and inputs
		float offset = params[OFFSET_PARAM].getValue();
		offset *= 5.f;
		float minimum = -5.f + offset;
		float maximum = 5.f + offset;

		float frequencyControl = params[FREQUENCY_PARAM].getValue();
		frequencyControl += inputs[FREQUENCY_INPUT].getVoltage();
		frequencyControl = clamp(frequencyControl, -5.f, 5.f);

		float attackParam = params[ATTACK_PARAM].getValue();
		float attackInput = inputs[ATTACK_INPUT].getVoltage() * 0.1f;
		float attack = (pow(TIME_RATIO, attackParam) + pow(TIME_RATIO, attackInput)) * MINIMUM_TIME;

		float targetPosition = params[POSITION_PARAM].getValue();
		targetPosition += offset;
		if (inputs[POSITION_INPUT].isConnected()) {
			targetPosition = inputs[POSITION_INPUT].getVoltage();
			targetPosition = clamp(targetPosition, minimum, maximum);
		}

		bool reset = resetTrigger.process(inputs[RESET_INPUT].getVoltage());

		// trace back given variables to coefficients
		float frequency = 2.f * pow(2, frequencyControl);
		float angularVelocity = 2 * M_PI * frequency;
		float stiffness = pow(angularVelocity, 2);

		float decay = -LOG_UNNOTICEABLE_DIFFERENCE / attack;
		float dampingRatio = decay / angularVelocity;
		float friction = 2 * sqrt(stiffness) * dampingRatio;

		// simulate spring movement
		float difference = position - targetPosition;
		float tension = -stiffness * difference;
		float damping = -friction * velocity;
		float acceleration = tension + damping;
		velocity += acceleration * args.sampleTime;
		position += velocity * args.sampleTime;
		position = clamp(position, minimum, maximum);

		if (reset) {
			velocity = 0;
			position = targetPosition;
		}

		// normalize and write outputs and lights
		outputs[ACCELERATION_OUTPUT].setVoltage(clamp(acceleration, -5.f, 5.f));
		outputs[VELOCITY_OUTPUT].setVoltage(clamp(velocity, -5.f, 5.f));
		outputs[POSITION_OUTPUT].setVoltage(position);
		lights[DIFFERENCE_LIGHT].setBrightness(abs(difference) * 0.1f);
	}
};


struct SpringTheoryWidget : ModuleWidget {
	SpringTheoryWidget(SpringTheory* module) {
		setModule(module);
		setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/SpringTheory.svg")));

		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addInput(createInputCentered<PJ301MPort>(grid(3, 11), module, SpringTheory::FREQUENCY_INPUT));
		addParam(createParamCentered<RoundBlackKnob>(grid(8, 11), module, SpringTheory::FREQUENCY_PARAM));
		addInput(createInputCentered<PJ301MPort>(grid(3, 16), module, SpringTheory::ATTACK_INPUT));
		addParam(createParamCentered<RoundBlackKnob>(grid(8, 16), module, SpringTheory::ATTACK_PARAM));

		addParam(createLightParamCentered<LEDLightSlider<WhiteLight>>(grid(3, 34), module, SpringTheory::POSITION_PARAM, SpringTheory::DIFFERENCE_LIGHT));
		addParam(createParamCentered<CKSS>(grid(6, 44), module, SpringTheory::OFFSET_PARAM));
		addInput(createInputCentered<PJ301MPort>(grid(3, 44), module, SpringTheory::POSITION_INPUT));

		addInput(createInputCentered<PJ301MPort>(grid(9, 29), module, SpringTheory::RESET_INPUT));
		addOutput(createOutputCentered<PJ301MPort>(grid(9, 34), module, SpringTheory::ACCELERATION_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(grid(9, 39), module, SpringTheory::VELOCITY_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(grid(9, 44), module, SpringTheory::POSITION_OUTPUT));
	}
};


Model* modelSpringTheory = createModel<SpringTheory, SpringTheoryWidget>("SpringTheory");
