
PROJECT = run-uart

OBJECTS += main.o blerssi.o

WORKSPACE_PATH = ../

PROJECT_PATH := $(patsubst %/Makefile,%,$(realpath $(lastword $(MAKEFILE_LIST))))
PROJECT_PATH := $(patsubst %/makefile,%,$(PROJECT_PATH))

include $(WORKSPACE_PATH)Makefile
