#ifndef ALPHAFILTER_HPP
#define ALPHAFILTER_HPP

#include <cassert>
#include <iostream>
#include <string>
#include <set>
#include <vector>

class AlphaFilter {
public:
    /**
     * @brief AlphaFilter alpha filter for joint position values
     * @param target_joints list of joint names that should be filtered
     * @param alpha the alpha value for the filter in the range 0..1
     */
    AlphaFilter(const std::set<std::string> &target_joints, float alpha);

    /**
     * @brief getAlpha get alpha value
     * @return alpha value
     */
    double getAlpha() { return _alpha; }

    /**
     * @brief update update the internal state of the filter by new measurements
     * The method will exchange the provided values with the filtered values in-place.
     * @param name full list of joint names as provided from the LCM message
     * @param measurement new measured joint positions
     */
    void update(const std::vector<std::string> &name, std::vector<float> &measurement);

private:
    /**
     * @brief setup set-up the filter by obtaining the target indices and initial values
     * Call this method once (e.g. check if isConfigured returns false) to obtain the
     * indices of the target joints in the full joint vector and to set the initial
     * filtered value x_0. After calling this method, isConfigured will return true.
     * @param name full list of joint names as provided from the LCM message
     * @param values full list of joint position values as provided from the LCM message
     */
    void setup(const std::vector<std::string> &name, const std::vector<float> &values);

    /**
     * @brief isConfigured check if setup was already called, e.g. if the target indices and initial values are known
     * @return true if setup was called before
     * @return false if setup has not been called before
     */
    bool isConfigured() { return _configured; }

    bool _configured;       //!< flag to indicated if setup has been called
    double _alpha;          //!< alpha value of filter
    const std::set<std::string> _target_joints; //!< list of joints to be filtered
    std::vector<unsigned int> _target_index;    //!< indices of filtered joints in full joints
    std::vector<float> _values; //!< internal state of filter, e.g. the filtered values
};

#endif // ALPHAFILTER_HPP
