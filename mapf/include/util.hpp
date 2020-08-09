/*
 * util functions
 */

#pragma once
#include <random>
#include <chrono>

// for computation time
using Time = std::chrono::system_clock;


static void halt(const std::string& msg,
                 const char* caller=__builtin_FUNCTION(),
                 const char* filename=__builtin_FILE())
{
  std::cout << "error@" << caller
            << " in " << filename
            << ": ";
  std::cout << msg << std::endl;
  std::exit(1);
}

static void warn(const std::string& msg,
                 const char* caller=__builtin_FUNCTION())
{
  std::cout << "warn@" << caller << ": ";
  std::cout << msg << std::endl;
}

// whether element 'a' is found in vector T
template <typename T>
static bool inArray(const T a, const std::vector<T> &arr) {
  auto itr = std::find(arr.begin(), arr.end(), a);
  return itr != arr.end();
}

// return true or false
static bool getRandomBoolean(std::mt19937* MT) {
  std::uniform_int_distribution<int> r(0, 1);
  return r(*MT);
}

// return [from, to]
static int getRandomInt(int from, int to, std::mt19937* MT) {
  std::uniform_int_distribution<int> r(from, to);
  return r(*MT);
}

// return [from, to)
static float getRandomFloat(float from, float to, std::mt19937* MT) {
  std::uniform_real_distribution<float> r(from, to);
  return r(*MT);
}

// return one element randomly from vector
template <typename T>
static T randomChoose(std::vector<T> &arr, std::mt19937* MT) {
  return arr[getRandomInt(0, arr.size()-1, MT)];
}

// get elapsed time
static double getElapsedTime(const std::chrono::system_clock::time_point&
                             t_start)
{
  Time::time_point t_end = Time::now();
  return std::chrono::duration_cast<std::chrono::milliseconds>
    (t_end-t_start).count();
}
