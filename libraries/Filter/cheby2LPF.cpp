#include "cheby2LPF.h"


////////////////////////////////////////////////////////////////////////////////////////////
// user_DigitalLPF
////////////////////////////////////////////////////////////////////////////////////////////

template <class T>
user_DigitalLPF<T>::user_DigitalLPF() {
  _delay_element_1 = T();
  _delay_element_2 = T();
}

template <class T>
T user_DigitalLPF<T>::apply(const T &sample, const struct biquad_params &params) {
    if(is_zero(params.cutoff_freq) || is_zero(params.sample_freq)) {
        return sample;
    }

    T delay_element_0 = sample - _delay_element_1 * params.a1 - _delay_element_2 * params.a2;
    T output = delay_element_0 * params.b0 + _delay_element_1 * params.b1 + _delay_element_2 * params.b2;

    _delay_element_2 = _delay_element_1;
    _delay_element_1 = delay_element_0;

    return output;
}

template <class T>
void user_DigitalLPF<T>::reset() { 
    _delay_element_1 = _delay_element_2 = T();
}

template <class T>
void user_DigitalLPF<T>::compute_params(float sample_freq, float cutoff_freq, biquad_params &ret) {
    ret.cutoff_freq = cutoff_freq;
    ret.sample_freq = sample_freq;
    if (!is_positive(ret.cutoff_freq)) {
        // zero cutoff means pass-thru
        return;
    }

    //Ceofficients are calulated using Matlab and simply hard-coded here.
    ret.b0 = 0.003284070226313f;
    ret.b1 = -0.005840003467515f;
    ret.b2 = 0.003284070226313f;
    ret.a1 = -1.961535747648295f;
    ret.a2 = 0.962263884633406f;
}


////////////////////////////////////////////////////////////////////////////////////////////
// cheby2LPF
////////////////////////////////////////////////////////////////////////////////////////////

template <class T>
cheby2LPF<T>::cheby2LPF() { 
    memset(&_params, 0, sizeof(_params) ); 
}

// constructor
template <class T>
cheby2LPF<T>::cheby2LPF(float sample_freq, float cutoff_freq) {
    // set initial parameters
    set_cutoff_frequency(sample_freq, cutoff_freq);
}

// change parameters
template <class T>
void cheby2LPF<T>::set_cutoff_frequency(float sample_freq, float cutoff_freq) {
    user_DigitalLPF<T>::compute_params(sample_freq, cutoff_freq, _params);
}

// return the cutoff frequency
template <class T>
float cheby2LPF<T>::get_cutoff_freq(void) const {
    return _params.cutoff_freq;
}

template <class T>
float cheby2LPF<T>::get_sample_freq(void) const {
    return _params.sample_freq;
}

template <class T>
T cheby2LPF<T>::apply(const T &sample) {
    if (!is_positive(_params.cutoff_freq)) {
        // zero cutoff means pass-thru
        return sample;
    }
    return _filter.apply(sample, _params);
}

template <class T>
void cheby2LPF<T>::reset(void) {
    return _filter.reset();
}

/* 
 * Make an instances
 * Otherwise we have to move the constructor implementations to the header file :P
 */
template class cheby2LPF<int>;
template class cheby2LPF<long>;
template class cheby2LPF<float>;
template class cheby2LPF<Vector2f>;
template class cheby2LPF<Vector3f>;
