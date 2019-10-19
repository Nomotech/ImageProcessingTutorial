#ifndef SHADER_HPP
#define SHADER_HPP

GLuint LoadShaders(const char* vertex_file_path, const char* fragment_file_path);
GLuint LoadCompShader(const char* compute_file_path);
//template< typename T, typename Alloc >
//GLuint makeGLBuffer(GLenum const type, GLenum const usage, std::vector< T, Alloc > const& vec);

#endif