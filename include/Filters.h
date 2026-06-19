#pragma once

#include <Arduino.h>
#include <stdint.h>

template <typename T, size_t Size>
class MovingAverageFilter
{
public:
  MovingAverageFilter() : head(0), count(0), sum(0)
  {
    for (size_t i = 0; i < Size; ++i)
    {
      buffer[i] = 0;
    }
  }

  void add(T value)
  {
    if (count < Size)
    {
      buffer[head] = value;
      sum += value;
      count++;
    }
    else
    {
      sum -= buffer[head];
      buffer[head] = value;
      sum += value;
    }
    head = (head + 1) % Size;
  }

  T get() const
  {
    if (count == 0) return 0;
    return static_cast<T>(sum / count);
  }

  void reset()
  {
    head = 0;
    count = 0;
    sum = 0;
  }

private:
  T buffer[Size];
  size_t head;
  size_t count;
  double sum;
};

template <typename T, size_t Size>
class MedianFilter
{
public:
  MedianFilter() : head(0), count(0)
  {
    for (size_t i = 0; i < Size; ++i)
    {
      buffer[i] = 0;
    }
  }

  void add(T value)
  {
    buffer[head] = value;
    head = (head + 1) % Size;
    if (count < Size)
    {
      count++;
    }
  }

  T get() const
  {
    if (count == 0) return 0;

    T temp[Size];
    for (size_t i = 0; i < count; ++i)
    {
      temp[i] = buffer[i];
    }

    // Fast insertion sort to find median
    for (size_t i = 1; i < count; ++i)
    {
      T key = temp[i];
      int j = static_cast<int>(i) - 1;
      while (j >= 0 && temp[j] > key)
      {
        temp[j + 1] = temp[j];
        j--;
      }
      temp[j + 1] = key;
    }

    return temp[count / 2];
  }

  void reset()
  {
    head = 0;
    count = 0;
  }

private:
  T buffer[Size];
  size_t head;
  size_t count;
};
