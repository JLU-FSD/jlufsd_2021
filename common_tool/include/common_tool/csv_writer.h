#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>
namespace io
{
class CSVWriter
{
public:
	CSVWriter(const std::string filename)
	{
		out_file_.open(filename, std::ios::trunc | std::ios::out);
	}
	template <class T>
	void write_row(const std::vector<T> string_vector)
	{
		for (int i = 0; i < string_vector.size(); i++)
		{
			out_file_ << std::fixed << std::setprecision(12) << string_vector[i] << ',';
		}
		out_file_ << std::endl;
	}
	template <class T>
	void write_row(T *array, unsigned int size)
	{

		for (int i = 0; i < size; i++)
		{
			out_file_ << std::fixed << std::setprecision(12) << array[i] << ',';
		}
		out_file_ << std::endl;
	}
	template <class T>
	void write_rows(T *array, unsigned int rows, unsigned int clos)
	{
		std::cout << "i am here" << std::endl;
		for (unsigned int i = 0; i <= rows - 1; i++)
		{
			for (unsigned int j = 0; j <= clos - 1; j++)
			{

				out_file_ << std::fixed << std::setprecision(12) << array[i * clos + j] << ',';
			}
			out_file_ << std::endl;
		}
	}
	template <class T>
	void write_row(T single_t)
	{
		out_file_ << std::fixed << std::setprecision(12) << single_t << ',';
		out_file_ << std::endl;
	}
	~CSVWriter()
	{
		out_file_.close();
	}

private:
	std::ofstream out_file_;
};
} // namespace io
