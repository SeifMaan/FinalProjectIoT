import 'package:flutter/material.dart';
import 'dart:math';

class GamePage extends StatefulWidget {
  @override
  _GamePageState createState() => _GamePageState();
}

class _GamePageState extends State<GamePage> {
  final int rows = 3;
  final int cols = 3;
  late List<List<int?>> puzzle;
  late List<List<int?>> solution;
  int emptyRow = 2;
  int emptyCol = 2;

  @override
  void initState() {
    super.initState();
    _initializePuzzle();
  }

  void _initializePuzzle() {
    List<int> numbers = List.generate(rows * cols - 1, (i) => i + 1);
    numbers.shuffle(Random());

    puzzle = List.generate(rows, (i) {
      return List.generate(cols, (j) {
        int index = i * cols + j;
        if (index < numbers.length) {
          return numbers[index];
        } else {
          return null;
        }
      });
    });

    solution = List.generate(rows, (i) {
      return List.generate(cols, (j) {
        int number = i * cols + j + 1;
        if (number == rows * cols) {
          return null;
        } else {
          return number;
        }
      });
    });
  }

  void _moveTile(int row, int col) {
    if ((row == emptyRow && (col == emptyCol - 1 || col == emptyCol + 1)) ||
        (col == emptyCol && (row == emptyRow - 1 || row == emptyRow + 1))) {
      setState(() {
        puzzle[emptyRow][emptyCol] = puzzle[row][col];
        puzzle[row][col] = null;
        emptyRow = row;
        emptyCol = col;
      });
    }
  }

  Widget _buildTile(int? number, int row, int col) {
    return GestureDetector(
      onTap: () => _moveTile(row, col),
      child: Container(
        decoration: BoxDecoration(
          color: number == null ? Colors.white : Colors.blueAccent,
          borderRadius: BorderRadius.circular(8.0),
          border: Border.all(color: Colors.black.withOpacity(0.2)),
          boxShadow: [
            BoxShadow(
              color: Colors.black.withOpacity(0.1),
              spreadRadius: 1,
              blurRadius: 5,
              offset: Offset(0, 3),
            ),
          ],
        ),
        child: Center(
          child: number == null
              ? null
              : Text(
                  number.toString(),
                  style: TextStyle(
                    fontSize: 24,
                    color: Colors.white,
                    fontWeight: FontWeight.bold,
                  ),
                ),
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Puzzle Game'),
        backgroundColor: Colors.blueAccent,
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Text(
              'Puzzle Game',
              style: TextStyle(
                fontSize: 28,
                fontWeight: FontWeight.bold,
                color: Colors.blueAccent,
              ),
            ),
            SizedBox(height: 20),
            Text(
              'Tap a tile to move it. Try to solve the puzzle!',
              style: TextStyle(
                fontSize: 16,
                color: Colors.grey[700],
              ),
              textAlign: TextAlign.center,
            ),
            SizedBox(height: 20),
            Expanded(
              child: GridView.builder(
                gridDelegate: SliverGridDelegateWithFixedCrossAxisCount(
                  crossAxisCount: cols,
                  crossAxisSpacing: 8.0,
                  mainAxisSpacing: 8.0,
                ),
                itemCount: rows * cols,
                itemBuilder: (context, index) {
                  int row = index ~/ cols;
                  int col = index % cols;
                  return _buildTile(puzzle[row][col], row, col);
                },
              ),
            ),
          ],
        ),
      ),
    );
  }
}
