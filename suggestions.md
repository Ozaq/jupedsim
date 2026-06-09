## Suggested Additional clang-tidy Checks

Recommended low-noise checks to add first:

```yaml
Checks: >-
  ...,
  modernize-loop-convert,
  modernize-use-default-member-init,
  modernize-use-emplace,
  modernize-use-equals-default,
  modernize-use-nullptr,
  modernize-use-using,
  readability-avoid-const-params-in-decls,
  readability-braces-around-statements,
  readability-container-size-empty,
  readability-const-return-type,
  readability-inconsistent-declaration-parameter-name,
  readability-named-parameter,
  readability-redundant-member-init,
  readability-simplify-boolean-expr,
  readability-use-anyofallof
```

Rationale:

- `modernize-use-nullptr`: enforce `nullptr` instead of `0` or `NULL`.
- `modernize-use-using`: prefer `using` over `typedef`.
- `modernize-use-emplace`: prefer `emplace_back` when appropriate.
- `modernize-loop-convert`: encourage range-for loops.
- `modernize-use-default-member-init`: reduce constructor boilerplate.
- `modernize-use-equals-default`: make defaulted special members explicit.
- `readability-braces-around-statements`: avoid ambiguous single-line control flow.
- `readability-container-size-empty`: prefer `.empty()` over `.size() == 0`.
- `readability-named-parameter`: avoid anonymous declaration parameters.
- `readability-inconsistent-declaration-parameter-name`: keep declaration and definition parameter names aligned.
- `readability-simplify-boolean-expr`: simplify noisy boolean expressions.
- `readability-use-anyofallof`: replace manual loops with standard algorithms where clearer.

