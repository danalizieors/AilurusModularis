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
		RESET_INPUT,
		STIFFNESS_INPUT,
		FRICTION_INPUT,
		POSITION_INPUT,
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
		configParam(FRICTION_PARAM, 0.f, 10.f, 0.f, "Friction", "Ns/m");
		configParam(POSITION_PARAM, -5.f, 5.f, -5.f, "Position", "m");
	}

	float position = 0.f;
    float velocity = 0.f;

	void process(const ProcessArgs& args) override {
		float offset = params[OFFSET_PARAM].getValue();
		offset *= 5.f;
		float stiffness = params[STIFFNESS_PARAM].getValue();
		float friction = params[FRICTION_PARAM].getValue();
		float targetPosition = params[POSITION_PARAM].getValue();
		targetPosition += offset;

		float difference = position - targetPosition;
		float tension = -stiffness * difference;
		float damping = -friction * velocity;
		float acceleration = tension + damping;
		velocity += acceleration * args.sampleTime;
		position += velocity * args.sampleTime;

		float minimum = -5.f + offset;
		float maximum = 5.f + offset;
		position = clamp(position, minimum, maximum);

		outputs[ACCELERATION_OUTPUT].setVoltage(clamp(acceleration, -5.f, 5.f));
		outputs[VELOCITY_OUTPUT].setVoltage(clamp(velocity, -5.f, 5.f));
		outputs[POSITION_OUTPUT].setVoltage(position);
		lights[DIFFERENCE_LIGHT].setBrightness(std::abs(difference) * 0.1f);
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
