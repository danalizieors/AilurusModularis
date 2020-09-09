#include "plugin.hpp"

Vec grid(float x, float y) {
	float halfGrid = RACK_GRID_WIDTH * 0.5f;
	return Vec(halfGrid * x, halfGrid * y);
}

struct SpringTheory : Module {
	enum ParamIds {
		OFFSET_PARAM,
		STIFFNESS_PARAM,
		FRICTION_PARAM,
		POSITION_PARAM,
		NUM_PARAMS
	};
	enum InputIds {
		STIFFNESS_INPUT,
		FRICTION_INPUT,
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
		configParam(STIFFNESS_PARAM, 0.f, 10.f, 1.f, "Stiffness", "N/m");
		configParam(FRICTION_PARAM, 0.f, 10.f, 1.f, "Friction", "Ns/m");
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

		float stiffness = params[STIFFNESS_PARAM].getValue();
		stiffness += inputs[STIFFNESS_INPUT].getVoltage();
		stiffness = std::max(0.f, stiffness);

		float friction = params[FRICTION_PARAM].getValue();
		friction += inputs[FRICTION_INPUT].getVoltage();
		friction = std::max(0.f, friction);

		float targetPosition = params[POSITION_PARAM].getValue();
		targetPosition += offset;
		if (inputs[POSITION_INPUT].isConnected()) {
			targetPosition = inputs[POSITION_INPUT].getVoltage();
			targetPosition = clamp(targetPosition, minimum, maximum);
		}

		bool reset = resetTrigger.process(inputs[RESET_INPUT].getVoltage());

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

		addInput(createInputCentered<PJ301MPort>(grid(3, 11), module, SpringTheory::STIFFNESS_INPUT));
		addParam(createParamCentered<RoundBlackKnob>(grid(8, 11), module, SpringTheory::STIFFNESS_PARAM));
		addInput(createInputCentered<PJ301MPort>(grid(3, 16), module, SpringTheory::FRICTION_INPUT));
		addParam(createParamCentered<RoundBlackKnob>(grid(8, 16), module, SpringTheory::FRICTION_PARAM));

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
