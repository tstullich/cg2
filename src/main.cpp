#include "glad/glad.h"
#include <GLFW/glfw3.h>
#include <iostream>

#include "parser.hpp"

/**
 * Callback that will be used when a user resizes
 * the window that the OpenGL context runs in
 */
void framebufferSizeCallback(GLFWwindow *window, int width, int height) {
  glViewport(0, 0, width, height);
}

/**
 * Function that can be used to process keyboard input.
 * Currently only supports quitting when the ESC key is pressed
 */
void processInput(GLFWwindow *window) {
  if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, true);
  }
}

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cout << "No file name was supplied. Please provide an OFF file to be "
                 "processed"
              << std::endl;
    return -1;
  }

  // Testing the Parser here. More can be done here before starting GLFW
  Parser p;
  if (!p.open(argv[1])) {
    std::cout << "Unable to open file for reading '" << argv[1] << "'"
              << std::endl;
    return -1;
  }

  // Attempting to init GLFW
  if (!glfwInit()) {
    std::cout << "Failed to initialize GLFW!" << std::endl;
    return -1;
  }

  // Setting up OpenGL context
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  // Create the window to run our OpenGL program in
  GLFWwindow *window = glfwCreateWindow(800, 600, "CG2", NULL, NULL);
  if (window == NULL) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return -1;
  }

  glfwMakeContextCurrent(window);
  glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

  // Attempt to load the OpenGL extensions through GLAD
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    return -1;
  }

  // Set the OpenGL context window size
  glViewport(0, 0, 800, 600);

  // Do some work to test if everything works
  while (!glfwWindowShouldClose(window)) {
    processInput(window);

    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // Cleanup
  p.close();
  glfwTerminate();

  return 0;
}
