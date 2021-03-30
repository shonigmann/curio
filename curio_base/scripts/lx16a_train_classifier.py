#!/usr/bin/env python
# 
#   Software License Agreement (BSD-3-Clause)
#    
#   Copyright (c) 2019 Rhys Mainwaring
#   All rights reserved
#    
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
# 
#   1.  Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
# 
#   2.  Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials provided
#       with the distribution.
# 
#   3.  Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#  
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.
# 

''' Train the encoder classifier.
'''

import joblib
import math
import numpy as np
import pandas as pd

from sklearn.metrics import accuracy_score
from sklearn.model_selection import cross_validate
from sklearn.model_selection import train_test_split
from sklearn.pipeline import make_pipeline
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.tree import DecisionTreeClassifier

import rclpy
from rclpy.node import Node
from curio_base.utils import get_param_or_die, get_param_default

def main(args=None):    
    rclpy.init(args=args) 
    node = Node('lx16a_train_classifier')
    node.get_logger().info('Starting LX-16A classifier training')

    # Load parameters
    check_accuracy_score = get_param_default(node,'check_accuracy_score', False)
    check_cross_validation_score = get_param_default(node,'check_cross_validation_score', False)
    dataset_filename = get_param_or_die(node,'dataset_filename')
    classifier_filename = get_param_or_die(node,'classifier_filename')
    
    # Load data from CSV 
    node.get_logger().info('Loading dataset: {}'.format(dataset_filename))
    df_data = pd.read_csv(dataset_filename, index_col=0, compression='zip')

    # Extract numpy arrays
    node.get_logger().info('Selecting features and targets')
    df_X = df_data.iloc[:,:-1]
    df_y = df_data.iloc[:,-1:]
    X = df_X.values
    y = df_y.values.ravel()
    node.get_logger().info("X.shape: {}, y.shape {}".format(X.shape, y.shape))

    # Create training and test datasets
    node.get_logger().info('Creating training and test datasets')
    df_X_train, df_X_test, df_y_train, df_y_test = train_test_split(df_X, df_y, random_state=0)
    X_train = df_X_train.values
    y_train = df_y_train.values.ravel()
    X_test = df_X_test.values
    y_test = df_y_test.values.ravel()

    # Create a pipeline for the decision tree classifier
    node.get_logger().info('Creating decision tree pipeline')
    clf_pipe = make_pipeline(
        StandardScaler(),
        DecisionTreeClassifier(random_state=0)
    )

    # Fit the whole pipeline
    node.get_logger().info('Fitting model to training dataset')
    clf_pipe.fit(X_train, y_train)

    # Check accuracy score
    if check_accuracy_score:
        node.get_logger().info('Calculating accuracy score...')
        score = accuracy_score(clf_pipe.predict(X_test), y_test)
        node.get_logger().info('Accuracy score: {}'.format(score))
 
    # Cross validation score
    if check_cross_validation_score:
        node.get_logger().info('Calculating cross validation scores...')
        cv = cross_validate(clf_pipe, X, y, cv=5)
        node.get_logger().info('CV test score:  {}'.format(cv['test_score']))

    # Persist using joblib (better for larger models)
    node.get_logger().info('Writing classifier: {}'.format(classifier_filename))
    joblib.dump(clf_pipe, classifier_filename, protocol=2)

if __name__ == '__main__':
    main()