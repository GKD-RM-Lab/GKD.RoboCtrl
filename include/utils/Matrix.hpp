/**
 ******************************************************************************
 * @file    matrix.cpp/h
 * @brief   Matrix/vector calculation. 矩阵/向量运算
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */
//TODO:优化矩阵乘法
#pragma once

#include <cmath>
#include <cstdint>
#include <array>
#include <algorithm>

namespace roboctrl::utils
{
template <int _rows, int _cols, typename T = float>
    requires(std::is_arithmetic_v<T> && _rows > 0 && _cols > 0) //编译期检查是否为算数类型
class Matrix
{

public:
    /**
     * @brief Constructor without input data
     * @param
     */
    constexpr Matrix() = default;

    constexpr explicit Matrix(T val)
    {
        for (int i = 0; i < _rows; i++)
            std::fill(data[i].begin(), data[i].end(), val);
    }

    /**
     * @brief Constructor with input data
     * @param data    A 2-D array buffer that stores the data
     */
    constexpr explicit Matrix(const T *data) : Matrix()
    {
        for (int i = 0; i < _rows; i++)
            for (int j = 0; j < _cols; j++)
                this->data[i][j] = data[i * _cols + j];
    }

    /**
     * @brief      Copy Constructor
     * @param mat  The copied matrix
     */
    constexpr Matrix(const Matrix &mat) : Matrix()
    {
        this->data = mat.data;
    }

    constexpr Matrix(Matrix &&mat) noexcept : Matrix()
    {
        this->data = std::move(mat.data);
    }

    template <typename Ty>
        requires(std::is_arithmetic_v<Ty>)
    constexpr Matrix(const Matrix<_rows, _cols, Ty> &mat) : Matrix()
    {
        for (int i = 0; i < _rows; i++)
            for (int j = 0; j < _cols; j++)
                data[i][j] = static_cast<T>(mat.data[i][j]);
    }

    /**
     * @brief Destructor
     */
    ~Matrix() = default;

    /**
     * @brief returns the row size of the matrix
     * @return _rows  The row size of the matrix
     */
    [[nodiscard]] constexpr uint32_t rows() const
    {
        return _rows;
    }

    /**
     * @brief return the column size of the matrix
     * @return _cols  The column size of the matrix
     */
    [[nodiscard]] constexpr uint32_t cols() const
    {
        return _cols;
    }

    /**
     * @brief Return the element of the matrix
     * @param row  The row
     */
    constexpr std::array<T, _cols> &operator[](int row)
    {
        return data[row];
    }

    constexpr const std::array<T, _cols> &operator[](int row) const
    {
        return data[row];
    }

    /**
     * @brief Copy assignment of the matrix(row * size) instance
     * @param mat   The copied prototype
     * @return      *this matrix
     */
    constexpr Matrix &operator=(const Matrix &mat)
    {
        data = mat.data;
        return *this;
    }

    constexpr Matrix &operator=(Matrix &&mat) noexcept
    {
        data = std::move(mat.data);
        return *this;
    }

    /**
     * @brief      Additional operator of two matrices(row * size)
     * @param mat  The matrix on the right hand side
     * @note  This function returns itself as the result
     * @return     The sum of two matrices
     */
    constexpr Matrix &operator+=(const Matrix &mat)
    {
        for (int i = 0; i < _rows; i++)
            for (int j = 0; j < _cols; j++)
                data[i][j] += mat.data[i][j];

        return *this;
    }

    /**
     * @brief Substraction operator of two matrices(row * size)
     * @param mat The matrix on the left hand side
     * @note  This function returns itself as the result
     * @return    The difference of two matrices
     */
    constexpr Matrix &operator-=(const Matrix &mat)
    {
        for (int i = 0; i < _rows; i++)
            for (int j = 0; j < _cols; j++)
                data[i][j] -= mat.data[i][j];

        return *this;
    }

    /**
     * @brief Scalar operator of the matrix and a scaling factor
     * @param val The scaling factor
     * @note  This function returns itself as the result
     * @return    THe scaled matrix
     */
    template <typename Ty>
        requires(std::is_arithmetic_v<Ty>)
    constexpr Matrix &operator*=(Ty val)
    {
        for (int i = 0; i < _rows; i++)
            for (int j = 0; j < _cols; j++)
                data[i][j] *= val;

        return *this;
    }

    /**
     * @brief Scalar operator of the matrix and a division factor
     * @param val The division factor
     * @note  This function returns itself as the result
     * @retval    matrix / val
     * @return    The scaled matrix
     */
    template <typename Ty>
        requires(std::is_arithmetic_v<Ty>)
    constexpr Matrix &operator/=(Ty val)
    {
        for (int i = 0; i < _rows; i++)
            for (int j = 0; j < _cols; j++)
                data[i][j] /= val;

        return *this;
    }

    /**
     * @brief Additonal operator
     * @note This function doesn't return itself but instead a new matrix instance
     * @param mat The matrix on the right hand side
     * @return The sum of the additional matrix
     */
    constexpr Matrix operator+(const Matrix &mat) const
    {
        Matrix res;
        for (int i = 0; i < _rows; i++)
            for (int j = 0; j < _cols; j++)
                res.data[i][j] = data[i][j] + mat.data[i][j];

        return res;
    }

    /**
     * @brief Substraction matrix
     * @note This function does not return itself but instead a new matrix instance
     * @param mat matrix on the right hand side
     * @return The sum of the substracted matrix
     */
    constexpr Matrix operator-(const Matrix &mat) const
    {
        Matrix res;
        for (int i = 0; i < _rows; i++)
            for (int j = 0; j < _cols; j++)
                res.data[i][j] = data[i][j] - mat.data[i][j];

        return res;
    }

    /**
     * @brief Scalar operator of the matrix and a scaling factor
     * @param val The scaling factor
     * @note      This function does not return itself
     * @return    THe scaled matrix
     */
    template <typename Ty>
        requires(std::is_arithmetic_v<Ty>)
    constexpr Matrix operator*(Ty val) const
    {
        Matrix res;
        for (int i = 0; i < _rows; i++)
            for (int j = 0; j < _cols; j++)
                res.data[i][j] = data[i][j] * val;

        return res;
    }

    /**
     * @brief Scalar operator of the matrix and a scaling factor
     * @param val The scaling factor on the left hand side
     * @note      This function does not return itself
     * @note      This time the scaling factor is on the left hand side
     * @return    THe scaled matrix
     */
    template <typename Ty>
        requires(std::is_arithmetic_v<Ty>)
    constexpr friend Matrix operator*(Ty val, const Matrix &mat)
    {
        return mat * val;
    }

    /**
     * @brief Scalar operator of the matrix and a division factor
     * @param val The division factor
     * @note  This function returns itself as the result
     * @retval    matrix / val
     * @return    The scaled matrix
     */
    template <typename Ty>
        requires(std::is_arithmetic_v<Ty>)
    Matrix operator/(Ty val) const
    {
        Matrix res;
        for (int i = 0; i < _rows; i++)
            for (int j = 0; j < _cols; j++)
                res.data[i][j] = data[i][j] / val;

        return res;
    }

    /**
     * @brief The matrix multiplication
     * @param mat1 the matrix on the LHS
     * @param mat2 the matrix on the RHS
     * @return The multiplication result
     */
    template <int cols2>
    constexpr friend Matrix<_rows, cols2, T> operator*(const Matrix<_rows, _cols, T> &mat1,
                                                       const Matrix<_cols, cols2, T> &mat2)
    {
        Matrix<_rows, cols2, T> res;

        for (int i = 0; i < _rows; i++)
            for (int k = 0; k < _cols; k++)
                for (int j = 0; j < cols2; j++)
                    res[i][j] += mat1[i][k] * mat2[k][j];

        return res;
    }

    /**
     * @brief Compare whether two matrices are identical
     *
     */
    constexpr bool operator==(const Matrix &mat) const
    {
        return data == mat.data;
    }

    // Submatrix
    template <int rows, int cols> constexpr Matrix<rows, cols, T> block(int start_row, int start_col) const
    {
        Matrix<rows, cols, T> res;
        for (int i = 0; i < rows; i++)
            std::copy(data[i + start_row].begin() + start_col, data[i + start_row].begin() + start_col + cols,
                      res.data[i].begin());
        return res;
    }

    /**
     * @brief Return the specific row of the matrix
     * @param row The row index
     * @retval The row vector presented in the matrix from
     */
    constexpr Matrix<1, _cols, T> row(int row) const
    {
        return block<1, _cols, T>(row, 0);
    }

    /**
     * @brief Return the specific row of the matrix
     * @param col The column index
     * @retval The column vector presented in the matrix from
     */
    constexpr Matrix<_rows, 1, T> col(int col) const
    {
        return block<_rows, 1, T>(0, col);
    }

    /**
     * @brief Get the transpose of the matrix
     * @param
     * @retval the transposed matrix
     */
    constexpr Matrix<_cols, _rows> trans() const
    {
        Matrix<_cols, _rows> res;
        for (int i = 0; i < _rows; i++)
            for (int j = 0; j < _cols; j++)
                res[j][i] = data[i][j];

        return res;
    }

    /**
     * Clone the matrix
     * @return the clone of the matrix
     */
    constexpr Matrix clone() const
    {
        Matrix res;
        res.data = data;
        return res;
    }

    /**
     * @brief Get the trace of the matrix
     * @param
     * @retval The trace of the matrix
     */
    constexpr T trace() const
    {
        T res = 0;
        for (int i = 0; i < fmin(_rows, _cols); i++)
            res += data[i][i];
        return res;
    }

    /**
     * @brief Get the inverse of the matrix
     * @param
     * @retval The inverse of the matrix
     */
    constexpr Matrix inv() const
    {
        Matrix res = Matrix::eye();
        static_assert(_cols == _rows, "Matrix must be square");

        // 创建一个临时矩阵用于 LU 分解
        Matrix temp = this->clone();

        // 高斯-约当消元法
        for (int k = 0; k < _cols; k++)
        {
            // 检查主元是否为 0
            if (temp[k][k] == 0.0f)
            {
                return zeros();
            }

            // 归一化当前行
            T pivot = temp[k][k];
            for (int j = 0; j < _rows; j++)
            {
                temp[k][j] /= pivot;
                res[k][j] /= pivot;
            }

            // 消去其他行的当前列
            for (int i = 0; i < _rows; i++)
            {
                if (i != k)
                {
                    T factor = temp[i][k];
                    for (int j = 0; j < _rows; j++)
                    {
                        temp[i][j] -= factor * temp[k][j];
                        res[i][j] -= factor * res[k][j];
                    }
                }
            }
        }

        return res;
    }

    /*==============================================================*/
    // Static function
    /**
     * @brief Returns a _rows x _cols zero matrix
     * @tparam _rows The row size
     * @tparam _cols The column size
     * @retval The zero matrix
     */
    static constexpr Matrix zeros()
    {
        return Matrix(static_cast<T>(0));
    }

    /**
     * @brief Returns a _rows x _cols one matrix
     * @tparam _rows The row size
     * @tparam _cols The column size
     * @retval The one matrix
     */
    static constexpr Matrix ones()
    {
        return Matrix(static_cast<T>(1));
    }

    /**
     * @brief Returns a _rows * columns  matrix
     * @tparam _rows The row size
     * @tparam _cols The column size
     * @retval The identity matrix
     */
    static constexpr Matrix eye()
    {
        Matrix mat = Matrix::zeros();

        for (int i = 0; i < std::min(_rows, _cols); i++)
            mat[i][i] = 1;

        return mat;
    }

    /**
     * @brief Returns a _rows x _cols diagonal matrix
     * @tparam _rows The row size
     * @tparam _cols The column size
     * @param vec The diagnoal entries
     * @retval The diagnoanl matrix
     */
    static constexpr Matrix diag(Matrix<_rows, 1> vec)
    {
        Matrix res = Matrix::zeros();
        for (int i = 0; i < std::min(_rows, _cols); i++)
        {
            res[i][i] = vec[i][0];
        }
        return res;
    }

private:
    std::array<std::array<T, _cols>, _rows> data;

};

template <int r, int c> using Matrixf = Matrix<r, c, float>;
} // namespace Power
