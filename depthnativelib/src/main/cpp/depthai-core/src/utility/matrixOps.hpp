#pragma once

#include <iostream>
#include <vector>

namespace dai {
namespace matrix {
/**
 * @brief Matrix multiplication between two matrixs of shape (m x n) and (n x p) of type float. -> firstMatrix * secondMatrix
 *
 * @param firstMatrix - vector of vector of float of shape (m x n)
 * @param secondMatrix - vector of vector of float of shape (n x p)
 * @return std::vector<std::vector<float>> - product of firstMatrix * secondMatrix, a vector of vector of float of shape (m x p)
 */
std::vector<std::vector<float>> matMul(std::vector<std::vector<float>>& firstMatrix, std::vector<std::vector<float>>& secondMatrix) {
    std::vector<std::vector<float>> res;

    if(firstMatrix[0].size() != secondMatrix.size()) {
        throw std::runtime_error("Number of column of the first matrix should match with the number of rows of the second matrix ");
        // Return an empty vector
        return res;
    }

    // Initializing elements of matrix mult to 0.
    for(size_t i = 0; i < firstMatrix.size(); ++i) {
        std::vector<float> col_vec(secondMatrix[0].size(), 0);
        res.push_back(col_vec);
    }

    // Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
    for(size_t i = 0; i < firstMatrix.size(); ++i) {
        for(size_t j = 0; j < secondMatrix[0].size(); ++j) {
            for(size_t k = 0; k < firstMatrix[0].size(); ++k) {
                res[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
            }
        }
    }

    return res;
}

/*
 * Function to get cofactor of A[p][q] in temp. n is current
 * dimension of part of A that we are calculating cofactor for.
 */
static void getCofactor(std::vector<std::vector<float>>& A, std::vector<std::vector<float>>& temp, size_t p, size_t q, size_t n) {
    size_t i = 0, j = 0;

    // Looping for each element of the matrix
    for(size_t row = 0; row < n; ++row) {
        for(size_t col = 0; col < n; ++col) {
            //  Copying into temporary matrix only those element
            //  which are not in given row and column
            if(row != p && col != q) {
                temp[i][j++] = A[row][col];

                // Row is filled, so increase row index and
                // reset col index
                if(j == n - 1) {
                    j = 0;
                    ++i;
                }
            }
        }
    }
}

/* Recursive function for finding determinant of matrix.
   n is current dimension of A[][]. */
static float determinant(std::vector<std::vector<float>>& A, size_t n) {
    float D = 0;  // Initialize result

    //  Base case : if matrix contains single element
    if(n == 1) return A[0][0];

    std::vector<std::vector<float>> temp(n, std::vector<float>(n, 0));  // To store cofactors
    int sign = 1;                                                       // To store sign multiplier

    // Iterate for each element of first row
    for(size_t f = 0; f < n; ++f) {
        // Getting Cofactor of A[0][f]
        getCofactor(A, temp, 0, f, n);
        D += sign * A[0][f] * determinant(temp, n - 1);

        // terms are to be added with alternate sign
        sign = -sign;
    }

    return D;
}

// Function to get adjoint of A in adj.
static void adjoint(std::vector<std::vector<float>>& A, std::vector<std::vector<float>>& adj) {
    if(A.size() == 1) {
        adj[0][0] = 1;
        return;
    }

    // temp is used to store the final cofactors of A
    int sign = 1;
    std::vector<std::vector<float>> temp(A.size(), std::vector<float>(A.size(), 0));

    for(size_t i = 0; i < A.size(); ++i) {
        for(size_t j = 0; j < A.size(); ++j) {
            // Get cofactor of A[i][j]
            getCofactor(A, temp, i, j, A.size());

            // sign of adj[j][i] positive if sum of row
            // and column indexes is even.
            sign = ((i + j) % 2 == 0) ? 1 : -1;

            // Interchanging rows and columns to get the
            // transpose of the cofactor matrix
            adj[j][i] = (sign) * (determinant(temp, A.size() - 1));
        }
    }
}

// Function to calculate and store inverse, returns false if
// matrix is singular
bool matInv(std::vector<std::vector<float>>& A, std::vector<std::vector<float>>& inverse) {
    // Find determinant of A[][]
    if(A[0].size() != A.size()) {
        throw std::runtime_error("Not a Square Matrix ");
    }

    float det = determinant(A, A.size());
    if(det == 0) {
        // cout << "Singular matrix, can't find its inverse";
        return false;
    }

    // Find adjoint
    std::vector<std::vector<float>> adj(A.size(), std::vector<float>(A.size(), 0));
    adjoint(A, adj);

    std::vector<float> temp;
    // Find Inverse using formula "inverse(A) = adj(A)/det(A)"
    for(size_t i = 0; i < A.size(); ++i) {
        for(size_t j = 0; j < A.size(); ++j) {
            temp.push_back(adj[i][j] / det);
        }
        inverse.push_back(temp);
        temp.clear();
    }

    return true;
}

}  // namespace matrix
}  // namespace dai