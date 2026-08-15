def greet(name: str, punctuation: str = "!") -> str:
    return f"Hello, {name}{punctuation}"


if __name__ == "__main__":
    print(greet("GitHub", punctuation=" from go-koushun-dev"))
